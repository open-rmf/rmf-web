import asyncio
import logging
import os
import signal
import sys
import threading
from typing import Awaitable, Callable, List, Union

import rclpy
from fastapi import Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from tortoise import Tortoise

from . import routes
from .app_config import AppConfig, default_config
from .authenticator import JwtAuthenticator
from .dependencies import auth_scheme
from .fast_io import FastIO
from .gateway import RmfGateway
from .repositories import RmfRepository, StaticFilesRepository
from .rmf_io import HealthWatchdog, RmfBookKeeper, RmfEvents


class App(FastIO):
    def __init__(self, app_config: AppConfig):
        logger = logging.getLogger("app")
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
        logger.addHandler(handler)
        logger.setLevel(app_config.log_level)

        authenticator = (
            JwtAuthenticator(
                app_config.jwt_public_key,
                app_config.aud,
                app_config.iss,
            )
            if app_config.jwt_public_key
            else None
        )

        super().__init__(
            authenticator=authenticator,
            logger=logger,
            title="RMF API Server",
            dependencies=[Depends(auth_scheme(authenticator, app_config.oidc_url))],
        )

        self.fapi.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["*"],
        )
        os.makedirs(app_config.static_directory, exist_ok=True)
        self.fapi.mount(
            "/static",
            StaticFiles(directory=app_config.static_directory),
            name="static",
        )

        async def load_states(rmf_events: RmfEvents, rmf_repo: RmfRepository):
            logger.info("loading states from database...")

            door_states = await rmf_repo.query_door_states()
            for state in door_states:
                rmf_events.door_states.on_next(state)
            logger.info(f"loaded {len(door_states)} door states")

            door_health = await rmf_repo.query_door_health()
            for health in door_health:
                rmf_events.door_health.on_next(health)
            logger.info(f"loaded {len(door_health)} door health")

            lift_states = await rmf_repo.query_lift_states()
            for state in lift_states:
                rmf_events.lift_states.on_next(state)
            logger.info(f"loaded {len(lift_states)} lift states")

            lift_health = await rmf_repo.query_lift_health()
            for health in lift_health:
                rmf_events.lift_health.on_next(health)
            logger.info(f"loaded {len(lift_health)} lift health")

            dispenser_states = await rmf_repo.query_dispenser_states()
            for state in dispenser_states:
                rmf_events.dispenser_states.on_next(state)
            logger.info(f"loaded {len(dispenser_states)} dispenser states")

            dispenser_health = await rmf_repo.query_dispenser_health()
            for health in dispenser_health:
                rmf_events.dispenser_health.on_next(health)
            logger.info(f"loaded {len(dispenser_health)} dispenser health")

            ingestor_states = await rmf_repo.query_ingestor_states()
            for state in ingestor_states:
                rmf_events.ingestor_states.on_next(state)
            logger.info(f"loaded {len(ingestor_states)} ingestor states")

            ingestor_health = await rmf_repo.query_ingestor_health()
            for health in ingestor_health:
                rmf_events.ingestor_health.on_next(health)
            logger.info(f"loaded {len(ingestor_health)} ingestor health")

            fleet_states = await rmf_repo.query_fleet_states()
            for state in fleet_states:
                rmf_events.fleet_states.on_next(state)
            logger.info(f"loaded {len(fleet_states)} fleet states")

            robot_health = await rmf_repo.query_robot_health()
            for health in robot_health:
                rmf_events.robot_health.on_next(health)
            logger.info(f"loaded {len(robot_health)} robot health")

            task_summaries = await rmf_repo.query_task_summaries()
            for summary in task_summaries:
                rmf_events.task_summaries.on_next(summary)
            logger.info(f"loaded {len(task_summaries)} tasks")

            logger.info("successfully loaded all states")

        # will be called in reverse order on app shutdown
        shutdown_cbs: List[Callable[[], Union[None, Awaitable[None]]]] = []
        started = False

        rmf_events = RmfEvents()
        rmf_repo = RmfRepository()
        rmf_gateway: RmfGateway = None

        def rmf_gateway_dep():
            return rmf_gateway

        self.include_router(
            routes.BuildingMapRouter(rmf_events), prefix="/building_map"
        )
        self.include_router(
            routes.DoorsRouter(rmf_events, rmf_gateway_dep, rmf_repo), prefix="/doors"
        )
        self.include_router(
            routes.LiftsRouter(rmf_events, rmf_gateway_dep, rmf_repo), prefix="/lifts"
        )
        self.include_router(routes.TasksRouter(rmf_gateway_dep), prefix="/tasks")
        self.include_router(
            routes.DispensersRouter(rmf_events, rmf_repo), prefix="/dispensers"
        )
        self.include_router(
            routes.IngestorsRouter(rmf_events, rmf_repo), prefix="/ingestors"
        )
        self.include_router(routes.FleetsRouter(rmf_events, rmf_repo), prefix="/fleets")

        @self.fapi.on_event("startup")
        async def on_startup():
            if authenticator is None:
                logger.warning("authentication is disabled")

            nonlocal started
            if started:
                raise RuntimeError("starting the app multiple times is not supported")

            loop = asyncio.get_event_loop()

            # shutdown event is not called when the app crashes, this can cause the app to be
            # "locked up" as some dependencies like tortoise does not allow python to exit until
            # it is closed "gracefully".
            def on_signal(sig, frame):
                task = loop.create_task(on_shutdown())
                if not loop.is_running():
                    loop.run_until_complete(task)
                if sig == signal.SIGINT and prev_sigint:
                    prev_sigint(sig, frame)
                elif sig == signal.SIGTERM and prev_sigterm:
                    prev_sigterm(sig, frame)

            if threading.currentThread() is threading.main_thread():
                prev_sigint = signal.signal(signal.SIGINT, on_signal)
                prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

            await Tortoise.init(
                db_url=app_config.db_url,
                modules={"models": ["api_server.models.tortoise_models"]},
            )
            await Tortoise.generate_schemas()
            shutdown_cbs.append(Tortoise.close_connections())

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

            static_files = StaticFilesRepository(
                f"{app_config.public_url.geturl()}/static",
                app_config.static_directory,
                logger.getChild("static_files"),
            )
            nonlocal rmf_gateway
            rmf_gateway = RmfGateway(rmf_events, static_files)

            # Order is important here
            # 1. load states from db, this populate the sio/fast_io rooms with the latest data
            await load_states(rmf_events, rmf_repo)

            # 2. start the services after loading states so that the loaded states are not
            # used. Failing to do so will cause for example, book keeper to save the loaded states
            # back into the db and mess up health watchdog's heartbeat system.
            rmf_gateway.subscribe_all()
            shutdown_cbs.append(rmf_gateway.unsubscribe_all)
            rmf_bookkeeper = RmfBookKeeper(
                rmf_repo, rmf_events, logger=logger.getChild("BookKeeper")
            )
            await rmf_bookkeeper.start()
            shutdown_cbs.append(rmf_bookkeeper.stop())
            health_watchdog = HealthWatchdog(
                rmf_gateway.rmf_events,
                rmf_repo=rmf_bookkeeper.repo,
                logger=logger.getChild("HealthWatchdog"),
            )
            await health_watchdog.start()

            # 3. start spinning only after setting everything up so that subscriptions
            # are not "lost" and the spin thread is sleeping because there is no "work"
            stopping = False

            def spin():
                logger.info("start spinning rclpy node")
                # using spin_once instead of spin because events can get missed/significantly
                # delayed if they are added from another thread while the spin thread is sleeping
                while not stopping:
                    rclpy.spin_once(rmf_gateway, timeout_sec=0.1)
                logger.info("finished spinning rclpy node")

            spin_thread = threading.Thread(target=spin)
            spin_thread.start()

            def stop_spinning():
                nonlocal stopping
                stopping = True
                spin_thread.join()

            shutdown_cbs.append(stop_spinning)

            started = True
            logger.info("started app")

        @self.fapi.on_event("shutdown")
        async def on_shutdown():
            while shutdown_cbs:
                cb = shutdown_cbs.pop()
                if asyncio.iscoroutine(cb):
                    await cb
                else:
                    cb()

            logger.info("shutdown app")


app = App(default_config)
