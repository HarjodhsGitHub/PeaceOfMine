# ArbotiX-M Servo Toolbox

This is a standalone static website. It has no backend, package install, or
Python runtime requirement. The browser connects directly to the ArbotiX FTDI
device through the Web Serial API.

## Use

1. Close any Python/terminal program that has the ArbotiX serial port open.
2. Open `index.html` in current Chrome or Edge on desktop.
3. Click **Connect ArbotiX** and choose the FTDI serial device in the browser
   permission picker.

If a browser blocks `file://` access in its local policy, host this directory
with any static HTTPS host or a local static-file server. The host must not
interfere with the serial connection; all DYNAMIXEL traffic stays in the
browser.

Web Serial is not supported by Safari or Firefox. It requires a secure context
and a user-initiated permission request.

## Live telemetry

The toolbox polls each selected servo and plots its MX-64 **Present Load**
register over the most recent 60 seconds. The value is a signed controller load
estimate displayed as -100% to +100%; it is not calibrated physical force,
pressure, or output-shaft torque.

The chart uses uPlot 1.6.32. Its JavaScript, stylesheet, and MIT license are
vendored in `web/vendor/`, so the page has no CDN or runtime dependency.
