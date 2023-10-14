import time
import network

ssidRouter     = 'SpectrumSetup-00' #Enter the router name Enter SSID here
passwordRouter = 'heartyocean483' #Enter the router password

def STA_Setup(ssidRouter,passwordRouter):
    print("Setup start")
    sta_if = network.WLAN(network.STA_IF)
    if not sta_if.isconnected():
        print('connecting to',ssidRouter)
        sta_if.active(True)
        sta_if.connect(ssidRouter,passwordRouter)
        while not sta_if.isconnected():
            pass
    print('Connected, IP address:', sta_if.ifconfig())
    print("Setup End and Ready to work ")

try:
    STA_Setup(ssidRouter,passwordRouter)    
except:
    sta_if.disconnect()
    