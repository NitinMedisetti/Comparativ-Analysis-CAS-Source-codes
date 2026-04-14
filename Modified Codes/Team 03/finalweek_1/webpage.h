#ifndef WEBPAGE_H
#define WEBPAGE_H

const char webpage[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8" />
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RoboDog Command</title>
<style>
/* --- APPLE-STYLE DARK MODE THEME --- */
:root {
    --bg-color: #1c1c1e;
    --card-bg: #2c2c2e;
    --text-primary: #ffffff;
    --text-secondary: #8e8e93;
    --accent-blue: #0a84ff;
    --accent-red: #ff453a;
    --accent-green: #30d158;
    --divider: #38383a;
}
body {
    background-color: var(--bg-color);
    color: var(--text-primary);
    font-family: -apple-system, BlinkMacSystemFont, "SF Pro Text", "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
    margin: 0; padding: 0;
    -webkit-font-smoothing: antialiased;
}
header {
    background: rgba(28, 28, 30, 0.8);
    backdrop-filter: blur(10px);
    position: sticky; top: 0; z-index: 100;
    text-align: center; padding: 15px 0;
    font-size: 20px; font-weight: 600;
    border-bottom: 1px solid var(--divider);
}
main {
    display: grid;
    grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
    gap: 15px; padding: 20px;
    max-width: 1200px; margin: 0 auto;
}

/* --- CARD STYLING --- */
.box {
    background: var(--card-bg);
    border-radius: 14px;
    padding: 20px;
    box-shadow: 0 4px 12px rgba(0,0,0,0.2);
}
h2 {
    font-size: 14px; color: var(--text-secondary); text-transform: uppercase;
    letter-spacing: 0.5px; margin: 0 0 15px 0; font-weight: 600;
}

/* --- DATA TABLES --- */
table { width: 100%; border-collapse: collapse; }
td { padding: 8px 0; font-size: 17px; border-bottom: 1px solid var(--divider); }
td:first-child { color: var(--text-secondary); font-weight: 400; }
td:last-child { text-align: right; font-weight: 500; font-variant-numeric: tabular-nums; }
tr:last-child td { border-bottom: none; }

/* --- CONTROLS --- */
.control-grid { display: flex; flex-direction: column; gap: 10px; }
.btn {
    background: var(--accent-blue); color: white; border: none;
    padding: 16px; border-radius: 12px; font-size: 16px; font-weight: 600;
    cursor: pointer; transition: opacity 0.2s; text-align: center;
}
.btn:active { opacity: 0.7; }
.btn-stop { background: var(--accent-red); }

/* --- CAMERA PLACEHOLDER --- */
#cameraBox {
    background: #000; height: 200px; border-radius: 12px;
    display: flex; align-items: center; justify-content: center;
    color: var(--text-secondary); font-size: 14px;
    border: 1px solid var(--divider);
}

</style>
</head>
<body>
<header>RoboDog Interface</header>
<main>
    <div class="box">
        <h2>GPS & Location</h2>
        <table>
            <tr><td>Latitude</td><td id="lat">--</td></tr>
            <tr><td>Longitude</td><td id="lon">--</td></tr>
            <tr><td>Altitude</td><td id="alt">--</td></tr>
            <tr><td>Satellites</td><td id="sats">--</td></tr>
            <tr><td>X</td><td id="x_local">--</td></tr>
            <tr><td>Y</td><td id="y_local">--</td></tr>
        </table>
    </div>

    <div class="box">
        <h2>PATHFINDING</h2>
        <table>
            <tr><td>Command</td><td id="pathCmd">--</td></tr>
            <tr><td>LEFT_WALL_ANGLE</td><td id="left">--</td></tr>
            <tr><td>RIGHT_WALL_ANGLE</td><td id="right">--</td></tr>
            <tr><td>FRONT_WALL_ANGLE</td><td id="frontld">--</td></tr>
            <tr><td>LEFT_WALL_DISTANCE</td><td id="left_dis">--</td></tr>
            <tr><td>RIGHT_WALL_DISTANCE</td><td id="right_dis">--</td></tr>
            <tr><td>FRONT_WALL_DISTANCE</td><td id="frontld_dis">--</td></tr>
        </table>
    </div>

    <div class="box">
        <h2>Orientation (IMU)</h2>
        <table>
            <tr><td>Heading</td><td id="yaw">--</td></tr>
            <tr><td>Pitch</td><td id="pitch">--</td></tr>
            <tr><td>Roll</td><td id="roll">--</td></tr>
        </table>
        </div>

    <div class="box">
        <h2>Ultrasonic (cm)</h2>
        <table>
            <tr><td>Front</td><td id="front">--</td></tr>
            <tr><td>Back</td><td id="back">--</td></tr>
            <tr><td>Left</td><td id="leftU">--</td></tr>
            <tr><td>Right</td><td id="rightU">--</td></tr>
            <tr><td>45° Diag</td><td id="deg45">--</td></tr>
        </table>
    </div>

    <div class="control-grid">
        <div style="display:flex; gap:10px;">
            <button class="btn" style="flex:1;" id="btnHBuilding">Navigate H-Building</button>
            <button class="btn" style="flex:1;" id="btnPostOffice">Navigate Post Office</button>
            <button class="btn" style="flex:1;" id="btnCheckup">Run Initial Checkup</button>
        </div>

        <button class="btn btn-stop" id="btnStop">EMERGENCY STOP</button>

        <button class="btn" style="background:#0a84ff;" 
                onclick="window.location.href='http://192.168.4.2/'">
            Open Camera Stream
        </button>
    </div>
</main>

<script>
let ws;

function sendCommand(cmd) {
    if (ws && ws.readyState === WebSocket.OPEN) ws.send(cmd);
}

function initWebSocket(){
    ws = new WebSocket('ws://' + window.location.hostname + ':81/');
    ws.onopen = () => console.log('Connected');
    ws.onmessage = (event) => {
        const data = event.data.split(',');
        if(data.length < 21) return;

        // Parse Core Data
        const roll = parseFloat(data[0]);
        const pitch = parseFloat(data[1]);
        const yaw = parseFloat(data[2]);
        
        // Update UI Text
        document.getElementById("yaw").innerText = yaw.toFixed(1) + "°";
        document.getElementById("pitch").innerText = pitch.toFixed(1) + "°";
        document.getElementById("roll").innerText = roll.toFixed(1) + "°";
        document.getElementById("lat").innerText = parseFloat(data[3]).toFixed(6);
        document.getElementById("lon").innerText = parseFloat(data[4]).toFixed(6);
        document.getElementById("alt").innerText = parseFloat(data[5]).toFixed(2) + "m";
        document.getElementById("sats").innerText = parseFloat(data[11]).toFixed(0);                    
        document.getElementById("x_local").innerText = parseFloat(data[12]).toFixed(2) + "m";
        document.getElementById("y_local").innerText = parseFloat(data[13]).toFixed(2) + "m";
        document.getElementById("pathCmd").innerText = data[14];
        document.getElementById("left").innerText = parseFloat(data[15]).toFixed(2);
        document.getElementById("right").innerText = parseFloat(data[16]).toFixed(2);
        document.getElementById("frontld").innerText = parseFloat(data[17]).toFixed(2);
        document.getElementById("left_dis").innerText = parseFloat(data[18]).toFixed(2);
        document.getElementById("right_dis").innerText = parseFloat(data[19]).toFixed(2);
        document.getElementById("frontld_dis").innerText = parseFloat(data[20]).toFixed(2);

        
        // Update Ultrasonic
        const updateUS = (id, val) => document.getElementById(id).innerText = (val > 0 ? val : "--");
        updateUS("deg45", data[6]); updateUS("front", data[7]);
        updateUS("leftU", data[8]); updateUS("rightU", data[9]); updateUS("back", data[10]);

        // Cube rotation logic removed
    };
    ws.onclose = () => setTimeout(initWebSocket, 2000);
}


window.onload = () => {
    initWebSocket();
    document.getElementById("btnHBuilding").onclick = () => sendCommand("H BUILDING");
    document.getElementById("btnPostOffice").onclick = () => sendCommand("POST BUILDING");
    document.getElementById("btnStop").onclick = () => sendCommand("STOP");
    document.getElementById("btnCheckup").onclick = () => sendCommand("CHECKUP");
    
};
</script>
</body>
</html>
)rawliteral";

#endif