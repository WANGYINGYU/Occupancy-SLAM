#!/usr/bin/env python3
"""Offline WebGL/UI check, used when no connected Browser session is available."""
import argparse
import json
import os
from pathlib import Path
from playwright.sync_api import sync_playwright

p=argparse.ArgumentParser()
p.add_argument('--html',type=Path,required=True)
p.add_argument('--browsers',type=Path,required=True)
a=p.parse_args()
root=Path(os.environ['SDF_BA_RUN_ROOT'])
os.environ['PLAYWRIGHT_BROWSERS_PATH']=str(a.browsers)
errors=[]
with sync_playwright() as driver:
    context=driver.chromium.launch_persistent_context(str(root/'browser_profile'),headless=True,
        viewport={'width':1500,'height':960},args=['--no-sandbox','--disable-dev-shm-usage','--use-angle=swiftshader','--enable-unsafe-swiftshader'])
    page=context.pages[0]
    page.on('pageerror',lambda error:errors.append(str(error)))
    page.goto(a.html.resolve().as_uri(),wait_until='load',timeout=60000)
    page.wait_for_function("document.querySelector('.js-plotly-plot')?._fullLayout?.scene2 != null",timeout=30000)
    page.wait_for_function("document.querySelectorAll('canvas').length >= 2",timeout=30000)
    page.screenshot(path=str(root/'viewer_points.png'),full_page=True)
    initial=page.evaluate("() => {const g=document.querySelector('.js-plotly-plot');return {traces:g.data.length,visible:g.data.map(t=>t.visible??true),scenes:!!g._fullLayout.scene2,canvas:document.querySelectorAll('canvas').length}}")
    assert initial['traces']==6 and initial['visible']==[True,True,True,True,False,False],initial
    page.get_by_text('Occupancy p ≥ 0.6 / 占据顶点',exact=True).click()
    page.wait_for_function("document.querySelector('.js-plotly-plot').data[4].visible === true")
    page.screenshot(path=str(root/'viewer_occupancy.png'),full_page=True)
    switched=page.evaluate("document.querySelector('.js-plotly-plot').data.map(t=>t.visible??true)")
    assert switched==[False,False,True,True,True,True],switched
    page.get_by_text('Point clouds / 点云',exact=True).click()
    # Exercise drag rotation and confirm the paired scene camera is synchronized.
    page.mouse.move(360,420);page.mouse.down();page.mouse.move(470,465,steps=12);page.mouse.up()
    page.wait_for_function("() => {const l=document.querySelector('.js-plotly-plot').layout;return JSON.stringify(l.scene.camera)===JSON.stringify(l.scene2.camera)}")
    camera=page.evaluate("document.querySelector('.js-plotly-plot').layout.scene.camera")
    assert abs(camera['eye']['x']-1.8)>1e-3 or abs(camera['eye']['y']+1.8)>1e-3,camera
    page.screenshot(path=str(root/'viewer_rotated.png'),full_page=True)
    assert not errors,errors
    (root/'ui_check.json').write_text(json.dumps({'initial':initial,'occupancy_visible':switched,'camera_after_drag':camera,'page_errors':errors,'passed':True},indent=2))
    context.close()
print('Offline viewer render, occupancy toggle and camera sync passed')
