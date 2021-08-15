from RobotRaconteur.Client import *
import json
import io
import sys
import zipfile
import os
import importlib
import asyncio
import traceback

import js



async def read_url(url):

    # Based on pyodide micropip module

    res = await js.fetch(url, {"cache": "no-store"})
    return io.BytesIO(await res.arrayBuffer())

async def download_install_webui_wheel(wheel_name):
    
    wheel_io = await read_url(f"/wheels/{wheel_name}.whl")
    with zipfile.ZipFile(wheel_io) as zf:
        zf.extractall(WHEEL_BASE)

async def load_wheels(wheels):
    for w in wheels:
        await download_install_webui_wheel(w)

    importlib.invalidate_caches()

async def bootstrap():
    try:
        config_json_text = (await read_url("/config")).read()
        config = json.loads(config_json_text)
        await load_wheels(config["wheels"])

        RRN.SetLogLevelFromString("INFO")
        RR.SetPythonTracebackPrintExc(True)

        from pyri.webui_browser import PyriWebUIBrowser

        pyri_webui = PyriWebUIBrowser(loop, config)
        loop.create_task(pyri_webui.run())
    except:
        traceback.print_exc()


for __p in sys.path:
    if __p.endswith('site-packages'):
        WHEEL_BASE = __p
        break

loop = asyncio.get_event_loop()
loop.create_task(bootstrap())