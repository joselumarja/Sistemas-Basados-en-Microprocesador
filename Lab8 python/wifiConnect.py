import network
 
def connect():
  ssid = "wifi2"
  password = "holahola"
 
  station = network.WLAN(network.STA_IF)
 
  if station.isconnected() == True:
      print("Already connected")
      return
 
  station.active(True)
  station.connect(ssid, password)
 
  while station.isconnected() == False:
      pass
 
  print("Connection successful")
  ip = station.ifconfig()
  print(ip)
  return ip[0]

