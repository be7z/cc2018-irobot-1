import picamera

camera = picamera.PiCamera()

camera.start_preview()
camera.capture('snapshot.jpg')
camera.stop_preview()
