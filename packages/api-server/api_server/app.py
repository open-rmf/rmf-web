import asyncio
import logging
import os
import signal
import sys
import threading
from typing import Any, Callable, Coroutine, List, Union

import rclpy
import rclpy.executors
import socketio
from fastapi import HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from api_server.types import is_coroutine

from . import routes
from .app_config import AppConfig, load_config
from .authenticator import JwtAuthenticator, StubAuthenticator
from .base_app import BaseApp
from .dependencies import rmf_repo as rmf_repo_dep
from .fast_io import FastIO
from .gateway import RmfGateway
from .models import (
    DispenserHealth,
    DispenserState,
    DoorHealth,
    DoorState,
    FleetState,
    IngestorHealth,
    IngestorState,
    LiftHealth,
    LiftState,
    RobotHealth,
)
from .models import tortoise_models as ttm
from .repositories import StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, RmfEvents


class App(FastIO, BaseApp):
    def __init__(
        self,
        *,
        app_config: AppConfig = None,
        rmf_gateway_fc: Callable[
            [RmfEvents, StaticFilesRepository], RmfGateway
        ] = RmfGateway,
    ):
        self.app_config = app_config or load_config(
            os.environ.get(
                "RMF_API_SERVER_CONFIG",
                f"{os.path.dirname(__file__)}/default_config.py",
            )
        )

        self.loop: asyncio.AbstractEventLoop
        logger = logging.getLogger("app")
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
        logger.addHandler(handler)
        logger.setLevel(self.app_config.log_level)

        if self.app_config.jwt_public_key:
            if self.app_config.iss is None:
                raise ValueError("iss is required")
            authenticator = JwtAuthenticator(
                self.app_config.jwt_public_key,
                self.app_config.aud,
                self.app_config.iss,
                oidc_url=self.app_config.oidc_url or "",
            )
        else:
            authenticator = StubAuthenticator()
            logger.warning("authentication is disabled")

        sio = socketio.AsyncServer(
            async_mode="asgi", cors_allowed_origins="*", logger=logger
        )

        super().__init__(
            sio,
            authenticator=authenticator,
            logger=logger,
            title="RMF API Server",
        )

        self.fapi.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        os.makedirs(self.app_config.static_directory, exist_ok=True)
        self.fapi.mount(
            "/static",
            StaticFiles(directory=self.app_config.static_directory),
            name="static",
        )

        # will be called in reverse order on app shutdown
        shutdown_cbs: List[Union[Coroutine[Any, Any, Any], Callable[[], None]]] = []

        self._rmf_events = RmfEvents()
        self.rmf_repo = rmf_repo_dep(self.auth_dep)
        self.static_files_repo = StaticFilesRepository(
            f"{self.app_config.public_url.geturl()}/static",
            self.app_config.static_directory,
            self.logger.getChild("static_files"),
        )
        self._rmf_gateway: RmfGateway

        self._rmf_bookkeeper = RmfBookKeeper(
            self._rmf_events, logger=self.logger.getChild("BookKeeper")
        )

        self.fapi.include_router(routes.main_router(self))
        self.include_router(routes.BuildingMapRouter(self), prefix="/building_map")
        self.include_router(routes.DoorsRouter(self), prefix="/doors")
        self.include_router(routes.LiftsRouter(self), prefix="/lifts")
        self.include_router(routes.TasksRouter(self), prefix="/tasks")
        self.include_router(routes.DispensersRouter(self), prefix="/dispensers")
        self.include_router(routes.IngestorsRouter(self), prefix="/ingestors")
        self.include_router(routes.FleetsRouter(self), prefix="/fleets")
        self.fapi.include_router(routes.admin_router(self), prefix="/admin")

        @self.fapi.on_event("startup")
        async def on_startup():
            self.loop = asyncio.get_event_loop()

            # shutdown event is not called when the app crashes, this can cause the app to be
            # "locked up" as some dependencies like tortoise does not allow python to exit until
            # it is closed "gracefully".
            def on_signal(sig, frame):
                task = self.loop.create_task(on_shutdown())
                if not self.loop.is_running():
                    self.loop.run_until_complete(task)
                if sig == signal.SIGINT and callable(prev_sigint):
                    prev_sigint(sig, frame)
                elif sig == signal.SIGTERM and callable(prev_sigterm):
                    prev_sigterm(sig, frame)

            if threading.currentThread() is threading.main_thread():
                prev_sigint = signal.signal(signal.SIGINT, on_signal)
                prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

            await Tortoise.init(
                db_url=self.app_config.db_url,
                modules={"models": ["api_server.models.tortoise_models"]},
            )
            await Tortoise.generate_schemas()
            shutdown_cbs.append(Tortoise.close_connections())

            await ttm.User.update_or_create(
                {"is_admin": True}, username=self.app_config.builtin_admin
            )

            use_sim_time_env = os.environ.get("RMF_SERVER_USE_SIM_TIME", None)
            if use_sim_time_env:
                use_sim_time = not use_sim_time_env.lower() in ["0", "false"]
            else:
                use_sim_time = False
            if use_sim_time:
                rclpy.init(args=["--ros-args", "-p", "use_sim_time:=true"])
            else:
                rclpy.init()
            shutdown_cbs.append(rclpy.shutdown)

            self._rmf_gateway = rmf_gateway_fc(self._rmf_events, self.static_files_repo)

            self._rmf_gateway.spin_background()
            shutdown_cbs.append(self._rmf_gateway.stop_spinning)

            # Order is important here
            # 1. load states from db, this populate the sio/fast_io rooms with the latest data
            await self._load_states()

            # 2. start the services after loading states so that the loaded states are not
            # used. Failing to do so will cause for example, book keeper to save the loaded states
            # back into the db and mess up health watchdog's heartbeat system.

            await self._rmf_bookkeeper.start()
            shutdown_cbs.append(self._rmf_bookkeeper.stop())
            health_watchdog = HealthWatchdog(
                self._rmf_events,
                logger=self.logger.getChild("HealthWatchdog"),
            )
            await health_watchdog.start()

            self._rmf_gateway.subscribe_all()
            shutdown_cbs.append(self._rmf_gateway.unsubscribe_all)

            self.logger.info("started app")

        @self.fapi.on_event("shutdown")
        async def on_shutdown():
            while shutdown_cbs:
                cb = shutdown_cbs.pop()
                if is_coroutine(cb):
                    await cb
                elif callable(cb):
                    cb()

            self.logger.info("shutdown app")

    async def _load_states(self):
        self.logger.info("loading states from database...")

        door_states = [DoorState.from_tortoise(x) for x in await ttm.DoorState.all()]
        for state in door_states:
            self._rmf_events.door_states.on_next(state)
        self.logger.info(f"loaded {len(door_states)} door states")

        door_health = [
            await DoorHealth.from_tortoise(x) for x in await ttm.DoorHealth.all()
        ]
        for health in door_health:
            self._rmf_events.door_health.on_next(health)
        self.logger.info(f"loaded {len(door_health)} door health")

        lift_states = [LiftState.from_tortoise(x) for x in await ttm.LiftState.all()]
        for state in lift_states:
            self._rmf_events.lift_states.on_next(state)
        self.logger.info(f"loaded {len(lift_states)} lift states")

        lift_health = [
            await LiftHealth.from_tortoise(x) for x in await ttm.LiftHealth.all()
        ]
        for health in lift_health:
            self._rmf_events.lift_health.on_next(health)
        self.logger.info(f"loaded {len(lift_health)} lift health")

        dispenser_states = [
            DispenserState.from_tortoise(x) for x in await ttm.DispenserState.all()
        ]
        for state in dispenser_states:
            self._rmf_events.dispenser_states.on_next(state)
        self.logger.info(f"loaded {len(dispenser_states)} dispenser states")

        dispenser_health = [
            await DispenserHealth.from_tortoise(x)
            for x in await ttm.DispenserHealth.all()
        ]
        for health in dispenser_health:
            self._rmf_events.dispenser_health.on_next(health)
        self.logger.info(f"loaded {len(dispenser_health)} dispenser health")

        ingestor_states = [
            IngestorState.from_tortoise(x) for x in await ttm.IngestorState.all()
        ]
        for state in ingestor_states:
            self._rmf_events.ingestor_states.on_next(state)
        self.logger.info(f"loaded {len(ingestor_states)} ingestor states")

        ingestor_health = [
            await IngestorHealth.from_tortoise(x)
            for x in await ttm.IngestorHealth.all()
        ]
        for health in ingestor_health:
            self._rmf_events.ingestor_health.on_next(health)
        self.logger.info(f"loaded {len(ingestor_health)} ingestor health")

        fleet_states = [FleetState.from_tortoise(x) for x in await ttm.FleetState.all()]
        for state in fleet_states:
            self._rmf_events.fleet_states.on_next(state)
        self.logger.info(f"loaded {len(fleet_states)} fleet states")

        robot_health = [
            await RobotHealth.from_tortoise(x) for x in await ttm.RobotHealth.all()
        ]
        for health in robot_health:
            self._rmf_events.robot_health.on_next(health)
        self.logger.info(f"loaded {len(robot_health)} robot health")

        self.logger.info("successfully loaded all states")

    def rmf_events(self) -> RmfEvents:
        return self._rmf_events

    def rmf_gateway(self) -> RmfGateway:
        return self._rmf_gateway

    def rmf_bookkeeper(self) -> RmfBookKeeper:
        return self._rmf_bookkeeper
