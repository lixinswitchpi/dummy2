#!/usr/bin/env python3
import asyncio
import logging

import time

from nicegui import app, ui

from udpclient import UDPClient
from controls import controls

logging.getLogger('nicegui').setLevel(logging.ERROR)

ui.colors(primary='#6e93d6')

devices: dict[int, ui.element] = {}

ui.markdown('### Rboot GUI')
ui.markdown('Waiting for Rboot CAN2ETH Gateway devices to be connected...').bind_visibility_from(globals(), 'devices', lambda d: not d)
container = ui.row()

async def discovery_loop() -> None:
    # odrive.start_discovery(odrive.default_usb_search_path)
    client = UDPClient('192.168.8.88', 9999)
    while True:
        if True:#client.is_server_up():
            client.connect()
            client.start_receive_thread()
            with ui.column() as devices[client]:
                controls(client)
                break
        else: 
            print("not conneted")
            time.sleep(1)
        # await asyncio.wrap_future(client.is_server_up())


app.on_startup(discovery_loop())

ui.run(title='Rboot GUI')
