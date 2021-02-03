import asyncio
import os
from logging import Logger
from typing import Optional

import rclpy
from rclpy.node import Node as RosNode

from fastapi import APIRouter, HTTPException

from socketio.asyncio_client import AsyncClient as SioAsyncClient


def building_map_router(ros2_node: RosNode, sio_client: SioAsyncClient, logger: Logger):
    router = APIRouter()

    building_map: Optional[BuildingMap] = None

    @sio_client.on('connect')
    def on_connect():
        print('connected')

    # @router.on_event('startup')
    # async def on_startup():
    #     loop = asyncio.get_event_loop()

    #     nonlocal building_map_sub
    #     building_map_sub = ros2_node.create_subscription(
    #         BuildingMap,
    #         'map',
    #         _on_building_map,
    #         rclpy.qos.QoSProfile(
    #             history=rclpy.qos.HistoryPolicy.KEEP_LAST,
    #             depth=1,
    #             reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
    #             durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
    #     logger.info('subscribed to "map"')

    # @router.on_event('shutdown')
    # async def on_shutdown():
    #     if building_map_sub:
    #         building_map_sub.destroy()

    @router.get('')
    async def get_building_map():
        if not building_map:
            raise HTTPException(503, 'map unavailable')
        return building_map

    return router
