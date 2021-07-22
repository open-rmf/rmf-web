import asyncio
import logging
import os
import signal
import sys
import threading
import time
from typing import Awaitable, Callable, List, Optional, Union

import rclpy
from fastapi import HTTPException
from fastapi.middleware.cors import CORSMiddleware
from tortoise import Tortoise

from . import routes
from .app_config import AppConfig, default_config
from .fast_io import FastIO
from .gateway import RmfGateway
from .repositories import RmfRepository
from .rmf_io import RmfBookKeeper, RmfEvents


class App(FastIO):
    def __init__(self, app_config: AppConfig):
        self.loop: asyncio.AbstractEventLoop = None
        logger = logging.getLogger("app")
        handler = logging.StreamHandler(sys.stdout)
        handler.setFormatter(logging.Formatter(logging.BASIC_FORMAT))
        logger.addHandler(handler)
        logger.setLevel(app_config.log_level)

        self._started: Optional[asyncio.Future] = None

        super().__init__(
            logger=logger,
            title="RMF Task Scheduler",
        )

        self.fapi.add_middleware(
            CORSMiddleware,
            allow_origins=["*"],
            allow_credentials=False,
            allow_methods=["*"],
            allow_headers=["*"],
        )

        async def load_states(rmf_events: RmfGateway, rmf_repo: RmfRepository):
            logger.info("loading states from database...")
            rmf_events = self.rmf_gateway.rmf_events
            logger.info("updating tasks from RMF")
            try:
                # Sometimes the node has not finished discovery so we need to call
                # `wait_for_service` here.
                # As of rclpy 3.0, `wait_for_service` uses a blocking sleep in a loop so
                # using it is not recommended after the app has finish startup.
                ready = self.rmf_gateway.get_tasks_srv.wait_for_service(1)
                if not ready:
                    raise HTTPException(503, "ros service not ready")
                await self.rmf_gateway.update_tasks(rmf_repo)
            except HTTPException as e:
                logger.error(f"failed to update tasks from RMF ({e.detail})")

            logger.info("successfully loaded all states")

        # will be called in reverse order on app shutdown
        shutdown_cbs: List[Callable[[], Union[None, Awaitable[None]]]] = []

        self.rmf_events = RmfEvents()
        self.rmf_repo = RmfRepository()
        self.rmf_gateway: RmfGateway = None
        rmf_bookkeeper = RmfBookKeeper(
            self.rmf_repo, self.rmf_events, logger=logger.getChild("BookKeeper")
        )

        def rmf_gateway_dep():
            return self.rmf_gateway

        self.include_router(
            routes.TasksRouter(
                self.rmf_events, rmf_bookkeeper.bookkeeper_events, rmf_gateway_dep
            ),
            prefix="/tasks",
        )

        @self.fapi.on_event("startup")
        async def on_startup():
            if self._started is not None:
                raise RuntimeError("starting the app multiple times is not supported")

            self.loop = asyncio.get_event_loop()
            self._started = asyncio.Future()

            # shutdown event is not called when the app crashes, this can cause the app to be
            # "locked up" as some dependencies like tortoise does not allow python to exit until
            # it is closed "gracefully".

            def on_signal(sig, frame):
                task = self.loop.create_task(on_shutdown())
                if not self.loop.is_running():
                    self.loop.run_until_complete(task)
                if sig == signal.SIGINT and prev_sigint:
                    prev_sigint(sig, frame)
                elif sig == signal.SIGTERM and prev_sigterm:
                    prev_sigterm(sig, frame)

            if threading.currentThread() is threading.main_thread():
                prev_sigint = signal.signal(signal.SIGINT, on_signal)
                prev_sigterm = signal.signal(signal.SIGTERM, on_signal)

            await Tortoise.init(
                db_url=app_config.db_url,
                modules={"models": ["task_scheduler.models.tortoise"]},
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

            self.rmf_gateway = RmfGateway(self.rmf_events)

            stopping = False

            def spin():
                logger.info("start spinning rclpy node")
                # using spin_once instead of spin because events can get missed/significantly
                # delayed if they are added from another thread while the spin thread is sleeping
                while not stopping:
                    rclpy.spin_once(self.rmf_gateway, timeout_sec=0.1)
                logger.info("finished spinning rclpy node")

            spin_thread = threading.Thread(target=spin)
            spin_thread.start()

            def stop_spinning():
                nonlocal stopping
                stopping = True
                spin_thread.join()

            shutdown_cbs.append(stop_spinning)

            # Order is important here
            # 1. load states from db, this populate the sio/fast_io rooms with the latest data
            await load_states(self.rmf_gateway, self.rmf_repo)

            # 2. start the services after loading states so that the loaded states are not
            # used. Failing to do so will cause for example, book keeper to save the loaded states
            # back into the db and mess up health watchdog's heartbeat system.
            self.rmf_gateway.subscribe_all()
            shutdown_cbs.append(self.rmf_gateway.unsubscribe_all)
            await rmf_bookkeeper.start()
            shutdown_cbs.append(rmf_bookkeeper.stop())

            self._started.set_result(True)
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

    def wait_ready(self):
        while self._started is None or not self._started.done():
            time.sleep(0.1)
        return True


app = App(default_config)
