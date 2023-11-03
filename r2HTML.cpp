#include "r2HTML.h"
r2HTML::r2HTML()
{
}

String r2HTML::getPWMOutputSetupHTML()
{
    String myHtml = header("PWM Output Setup") + R"rawliteral(
    
    %Combo%
    <form id="temp">
    %data%
    </form>
    <script>
    )rawliteral" +
                    getInputScript("tc", "updateTextBox") +
                    getInputScript("cb", "updatecombo") +
                    getInputScript("btn", "home") +
                    getInputScript("sl", "updateSlider") +
                    R"rawliteral(if (!!window.EventSource) {
    var source = new EventSource('/events');
 
    source.addEventListener('pwmoutput', function(e) {
     console.log("pwmoutput", e.data);
     document.getElementById("temp").innerHTML = e.data;
    }, false);
    }
    </script>
    )rawliteral" + footer;

    int str_len = myHtml.length() + 1;
    // Prepare the character array (the buffer)
    char char_array[str_len];
    // Copy it over
    myHtml.toCharArray(char_array, str_len);
    return char_array;
}

String r2HTML::header(String title)
{
    return R"rawliteral(<!DOCTYPE HTML><html>
<head>
  <title>)rawliteral" +
           title + R"rawliteral(</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
   <style>
    )rawliteral" +
           style() + R"rawliteral(
    html {font-family: Arial; display: inline-block; text-align: center;}
    h2 {font-size: 2.0rem;}
    p {font-size: 2.0rem;}
    
    .switch {position: relative; display: inline-block; width: 120px; height: 68px} 
    .switch input {display: none}
    .slider {position: absolute; top: 0; left: 0; right: 0; bottom: 0; background-color: #ccc; border-radius: 6px}
    .slider:before {position: absolute; content: ""; height: 52px; width: 52px; left: 8px; bottom: 8px; background-color: #fff; -webkit-transition: .4s; transition: .4s; border-radius: 3px}
    input:checked+.slider {background-color: #b30000}
    input:checked+.slider:before {-webkit-transform: translateX(52px); -ms-transform: translateX(52px); transform: translateX(52px)}
     #container {
            
            height: 49vh;
            background-color: #333;
            display: flex;
            align-items: center;
            justify-content: center;
            overflow: hidden;
            border-radius: 7px;
            touch-action: none;
        }

        #item {
            width: 100px;
            height: 100px;
            background-color: rgb(245, 230, 99);
            border: 10px solid rgba(136, 136, 136, .5);
            touch-action: none;
            user-select: none;
        }

        #item:active {
            background-color: rgba(168, 218, 220, 1.00);
        }

        #item:hover {
            cursor: pointer;
            border-width: 20px;
        }

        #area {
            position: fixed;
            right: 0;
            top: 0;
        }
  </style>
 </head>
<body>)rawliteral" +
           menu() + R"rawliteral(<div style="margin-left:100px;padding:1px 16px;height:1000px;"><h2>)rawliteral" +
           title + R"rawliteral(</h2>)rawliteral";
}

String r2HTML::menuStyle()
{
   return R"rawliteral(ul {
  list-style-type: none;
  margin: 0;
  width: 25vw;
  height: 100vh;
  padding: 0;
  background-color: #f1f1f1;
  position: fixed;
  overflow: auto;
}

li a {
  display: block;
  color: #000;
  padding: 8px 16px;
  text-decoration: none;
}

li a.active {
  background-color: #04AA6D;
  color: white;
}

li a:hover:not(.active) {
  background-color: #555;
  color: white;
})rawliteral";
}

String r2HTML::menu()
{
    return R"rawliteral(<ul>
  <li><a href="/">Home</a></li>
  <li><a href="Outputs">Config Outputs</a></li>
  <li><a href="Controller">Set Controller MAC</a></li>
  <li><a href="update">Firmware Update</a></li>
</ul>)rawliteral";
}

String r2HTML::style()
{
    return menuStyle();
}

String r2HTML::getInputScript(String functionName, String handelerPath)
{
    String html = "function " + functionName + R"rawliteral((element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/)rawliteral" +
                  handelerPath + R"rawliteral(?output="+element.id+"&state="+element.value, true);
  xhr.send();
  })rawliteral";

    return html;
}

String r2HTML::getInputScriptMAC(String functionName, String handelerPath)
{
    String html = "function " + functionName + R"rawliteral((element) {
  var xhr = new XMLHttpRequest();
  xhr.open("GET", "/)rawliteral" +
                  handelerPath + R"rawliteral(?output="+element.id+"&state="+document.getElementById('mac').value, true);
  xhr.send();
  })rawliteral";

    return html;
}

String r2HTML::getpsControllerHTML()
{
    String myHtml = header("PS Controller MAC Setup") + R"rawliteral(
    %Combo%
    %data%
    <script>
    )rawliteral" +
                    getInputScriptMAC("btn", "savemac") +
                    R"rawliteral(
    </script>
    )rawliteral" + footer;

    
    return myHtml;
}

String r2HTML::getIndexHTML()
{

    String Html = header("R2D2 Motor Drive Test") + R"rawliteral(
  <script type="text/javascript">
        var JoyStick = function (t, e) { var i = void 0 === (e = e || {}).title ? "joystick" : e.title, n = void 0 === e.width ? 0 : e.width, o = void 0 === e.height ? 0 : e.height, h = void 0 === e.internalFillColor ? "#00AA00" : e.internalFillColor, r = void 0 === e.internalLineWidth ? 2 : e.internalLineWidth, d = void 0 === e.internalStrokeColor ? "#003300" : e.internalStrokeColor, a = void 0 === e.externalLineWidth ? 2 : e.externalLineWidth, l = void 0 === e.externalStrokeColor ? "#008000" : e.externalStrokeColor, c = document.getElementById(t), u = document.createElement("canvas"); u.id = i, 0 == n && (n = c.clientWidth), 0 == o && (o = c.clientHeight), u.width = n, u.height = o, c.appendChild(u); var s = u.getContext("2d"), f = 0, v = 2 * Math.PI, g = (u.width - 110) / 2, w = g + 5, C = g + 30, m = u.width / 2, p = u.height / 2, L = u.width / 10, E = -1 * L, S = u.height / 10, k = -1 * S, W = m, G = p; function x() { s.beginPath(), s.arc(m, p, C, 0, v, !1), s.lineWidth = a, s.strokeStyle = l, s.stroke() } function y() { s.beginPath(), W < g && (W = w), W + g > u.width && (W = u.width - w), G < g && (G = w), G + g > u.height && (G = u.height - w), s.arc(W, G, g, 0, v, !1); var t = s.createRadialGradient(m, p, 5, m, p, 200); t.addColorStop(0, h), t.addColorStop(1, d), s.fillStyle = t, s.fill(), s.lineWidth = r, s.strokeStyle = d, s.stroke() } "ontouchstart" in document.documentElement ? (u.addEventListener("touchstart", function (t) { f = 1 }, !1), u.addEventListener("touchmove", function (t) { t.preventDefault(), 1 == f && (W = t.touches[0].pageX, G = t.touches[0].pageY, W -= u.offsetLeft, G -= u.offsetTop, s.clearRect(0, 0, u.width, u.height), x(), y()) }, !1), u.addEventListener("touchend", function (t) { f = 0, W = m, G = p, s.clearRect(0, 0, u.width, u.height), x(), y() }, !1)) : (u.addEventListener("mousedown", function (t) { f = 1 }, !1), u.addEventListener("mousemove", function (t) { 1 == f && (W = t.pageX, G = t.pageY, W -= u.offsetLeft, G -= u.offsetTop, s.clearRect(0, 0, u.width, u.height), x(), y()) }, !1), u.addEventListener("mouseup", function (t) { f = 0, W = m, G = p, s.clearRect(0, 0, u.width, u.height), x(), y() }, !1)), x(), y(), this.GetWidth = function () { return u.width }, this.GetHeight = function () { return u.height }, this.GetPosX = function () { return W }, this.GetPosY = function () { return G }, this.GetX = function () { return ((W - m) / w * 100).toFixed() }, this.GetY = function () { return ((G - p) / w * 100 * -1).toFixed() }, this.GetDir = function () { var t = "", e = W - m, i = G - p; return i >= k && i <= S && (t = "C"), i < k && (t = "N"), i > S && (t = "S"), e < E && ("C" == t ? t = "W" : t += "W"), e > L && ("C" == t ? t = "E" : t += "E"), t } };
    </script>
    <div id='outerContainer'>
        <div id="joyDiv" style="width:200px;height:200px;margin:auto;"></div>

    </div>

    <script>
 var gateway = `ws://${window.location.hostname}/ws`;
  var ws;
  window.addEventListener('load', onLoad);
  function initWebSocket() {
    console.log('Trying to open a WebSocket connection...');
    ws = new WebSocket(gateway);
    ws.onopen    = onOpen;
    ws.onclose   = onClose;
    ws.onmessage = onMessage; // <-- add this line
  }
  function onOpen(event) {
    console.log('Connection opened');
    var x = document.getElementById("joyDiv");
     x.style.display = "Block";
  }
  function onClose(event) {
    console.log('Connection closed');
    setTimeout(initWebSocket, 2000);
    var x = document.getElementById("joyDiv");
     x.style.display = "none";
  }
  function onMessage(event) {
    var state;
    if (event.data == "1"){
      state = "ON";
    }
    else{
      state = "OFF";
    }
    document.getElementById('state').innerHTML = state;
  }
  function onLoad(event) {
    initWebSocket();
    
  }
  
        var lastText, lastSend, sendTimeout;
        // limit sending to one message every 10 ms
        // https://github.com/neonious/lowjs_esp32_examples/blob/master/neonious_one/cellphone_controlled_rc_car/www/index.html
        function send(txt) {
            var now = new Date().getTime();
            if (lastSend === undefined || now - lastSend >= 10) {
                try {
                    ws.send(txt);
                    lastSend = new Date().getTime();
                    return;
                } catch (e) {
                    console.log(e);
                }
            }
            lastText = txt;
            if (!sendTimeout) {
                var ms = lastSend !== undefined ? 10 - (now - lastSend) : 10;
                if (ms < 0)
                    ms = 0;
                sendTimeout = setTimeout(() => {
                    sendTimeout = null;
                    send(lastText);
                }, ms);
            }
        }

    </script>
    </div>
</body>

<script type="text/javascript">
    // Create JoyStick object into the DIV 'joyDiv'
    var joy = new JoyStick('joyDiv');
    var inputPosX = document.getElementById("posizioneX");
    var inputPosY = document.getElementById("posizioneY");
    var x = document.getElementById("X");
    var y = document.getElementById("Y");

    setInterval(function () { send(joy.GetX() +","+ joy.GetY()); }, 300);

</script>

</html>)rawliteral";

    return Html;
}

#if !defined(NO_GLOBAL_INSTANCES)
r2HTML R2D2HTML;
#endif
