# Final

These are the hard-pinned Arduino builds for the shared-hotspot setup.

## 001

- Proxy: `001/1_Proxy_001`
  - `PROXY_BOX_NUM 1`
  - Advertises `BOX_001`
  - Accepts `CONTROLLER_001`
  - Accepts `OV3660_CAM_001`
- Controller: `001/2_Controller_001`
  - `CONTROLLER_BOX_NUM 1`
  - Sends `X-Controller-Id: CONTROLLER_001`
  - Only binds to `BOX_001`
- Camera: `001/3_Eye_001`
  - `CAMERA_BOX_NUM 1`
  - Sends `X-Camera-Id: OV3660_CAM_001`
  - Only binds to `BOX_001`

## 002

- Proxy: `002/1_Proxy_002`
  - `PROXY_BOX_NUM 2`
  - Advertises `BOX_002`
  - Accepts `CONTROLLER_002`
  - Accepts `OV3660_CAM_002`
- Controller: `002/2_Controller_002_Keypad`
  - `CONTROLLER_BOX_NUM 2`
  - Sends `X-Controller-Id: CONTROLLER_002`
  - Only binds to `BOX_002`
- Camera: `002/3_Eye_002`
  - `CAMERA_BOX_NUM 2`
  - Sends `X-Camera-Id: OV3660_CAM_002`
  - Only binds to `BOX_002`

Open and flash the sketch inside the matching numeric group. Do not mix files between `001` and `002`.
