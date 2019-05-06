import picoweb
import wifiConnect
from machine import UART

ip = wifiConnect.connect()
uart = UART(2, 9600)                         # init with given baudrate
uart.init(9600, bits=8, parity=None, stop=1) # init with given parameters
app = picoweb.WebApp(__name__)

def sendData(data):
	uart.write(data)
#Change the HTTP endpoint in accordance with your group ID
@app.route("/red/2")
def red(req, resp):
    #Here your code
    sendData("20")

    # You can change the message
    yield from picoweb.start_response(resp, content_type = "text/html")
    yield from resp.awrite("Sending Red")
    yield from resp.aclose()


@app.route("/green/2")
def green(req, resp):
    #Here your code
    sendData("21")
    yield from picoweb.start_response(resp, content_type = "text/html")
    yield from resp.awrite("Sending Green")
    yield from resp.aclose()
    pass


@app.route("/restore/2")
def restore(req, resp):
    #Here your code
    sendData("22")
    yield from picoweb.start_response(resp, content_type = "text/html")
    yield from resp.awrite("Sending Restore")
    yield from resp.aclose()
    pass
    
#if __name__ == '__main__':
app.run(debug=True, host = ip, port = 80)

# To check: open a web browser and go to http://<ip>:80/red/<ID>
