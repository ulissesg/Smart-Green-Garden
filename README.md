# Smart Green Garden
  Project done with a ESP8266, which is a automation for a green garden using a humidity sensor to turn on/off a water pump that irrigate the garden.

  It also have some others functions, such as control it using a mqtt app with my phone and check the humidity level, or even use the google assistent(that was made with IFTTT) to control it, and the code can be updated via OTA, so i don't need to go in the green garden to update de code, it can be done via wifi.
  
  One other feature is the modes of operation, so you can chose if you want the system to work based on the humidity of the ground or to work with the time, watering the plants in a certain time, like every hour you water the plants, this can be very healpful to more sensitive plants.

  To control all the ON and OFF via wifi thing, i used adafruit broker, which is a mqtt broker.
