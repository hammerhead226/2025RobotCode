from flask import Flask, request, render_template_string
from networktables import NetworkTables
import threading

app = Flask(__name__)

# Initialize NetworkTables (set to your RoboRIO IP or use mDNS like 'roborio-TEAM-frc.local')
NetworkTables.initialize(server='127.0.0.1')  # localhost for simulation
nt = NetworkTables.getTable('Reef')

# --- HTML code here ---
HTML_PAGE = ''' 
<!DOCTYPE html>
<html>
<head>
  <title>Reef Control Panel</title>
  <style>
    body {
      font-family: sans-serif;
      background-color: #dff6ff;
      text-align: center;
    }

    .row {
      display: flex;
      justify-content: center;
      margin: 5px 0;
    }
      .column {
  display: flex;
  flex-direction: column; /* stacks vertically */
  align-items: flex-start;
}


h1 {
  transform: rotate(180deg); /* rotates counterclockwise */
  transform-origin: right top; /* adjust pivot point */
  position: absolute;
  top: 750px; /* tweak as needed */
  left: 0px; /* tweak as needed */
}
    .indent-1 { margin-left: 40px; }
    .indent-2 { margin-left: 80px; }
    .indent-3 { margin-left: 120px; }
.rotated-90 {
  transform: rotate(90deg);
}
.rotated-180 {
  transform: rotate(180deg);
}
.rotated-custom {
  transform: rotate(45deg); /* or any degree */
}
    .btn {
      margin: 2px;
      padding: 8px 12px;
      background-color: #fff;
      border: 2px dashed #000000;
      border-radius: 6px;
      cursor: pointer;
      font-size: 14px;
       z-index: 1px;
  position: relative;

    }
    
    .algae-btn {
   background-color: #00cc88;
  color: white;
  width: 75px;
  height: 75px;
  position: absolute; /* Key change */
  border: 2px dashed #FFFFFF;
  border-radius: 50%;
  cursor: pointer;
  font-size: 18px;
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 0;
  pointer-events: auto; 
}



 .reef-container {
  position: relative;
  width: 700px;  /* or fixed width, like 700px */
  margin: 0 auto;       /* centers horizontally */
  top: 100px;            /* optional vertical spacing */
}
 
.clicked {
  background-color: #00cc88; /* or any color you like */
  color: white;
}
.clicked-algae {
  background-color: #FF0000; /* or any color you like */
  color: white;
}
  </style>
</head>

<body>
<script>
  function sendCommand(cmd) {
    fetch('/command', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ command: cmd })
    });
  }

  document.addEventListener('DOMContentLoaded', () => {
    const buttons = document.querySelectorAll('.btn');
    buttons.forEach(button => {
      button.addEventListener('click', () => {
        sendCommand(button.textContent.trim());
      });
    });
  });
</script>

<script>
  document.addEventListener('DOMContentLoaded', () => {
    const buttons = document.querySelectorAll('.btn');
    buttons.forEach(button => {
      button.addEventListener('click', () => {
        button.classList.toggle('clicked');
      });
    });
  });
</script>
<script>
  document.addEventListener('DOMContentLoaded', () => {
    const buttons = document.querySelectorAll('.algae-btn');
    buttons.forEach(button => {
      button.addEventListener('click', () => {
        button.classList.toggle('clicked-algae');
      });
    });
  });
</script>

 
  <!--<h1>Reef Control Panel</h1>-->
  
  <div class="reef-container">

   <div class="column" style="transform: translateX(0px) translateY(80px);">
    
    <button class="btn">J4</button>
    
  </div>
  
   <div class="column" style="transform: translateX(50px) translateY(40px);">
    <button class="btn">I4</button>
    <button class="btn">J3</button>
  </div>
  <div class="column" style="transform: translateX(100px) translateY(0px);">
    <button class="btn">I3</button>
    <button class="btn">J2</button>
  </div>
  <div class="column" style="transform: translateX(150px) translateY(-40px);">
    <button class="btn">I2</button>
    <button class="btn">J1</button>
  </div>
  <div class="column" style="transform: translateX(200px) translateY(-80px);">
    <button class="btn">I1</button>
  </div>
  <div class="column" style="transform: translateX(300px) translateY(-320px);">
    <button class="btn">G4</button>
    <button class="btn">G3</button>
    <button class="btn">G2</button>
    <button class="btn">G1</button>
  </div>
  <div class="column" style="transform: translateX(350px) translateY(-480px);">
    <button class="btn">H4</button>
    <button class="btn">H3</button>
    <button class="btn">H2</button>
    <button class="btn">H1</button>
  </div>
  <div class="column" style="transform: translateX(450px) translateY(-440px);">
    <button class="btn">F1</button>
  </div>
  <div class="column" style="transform: translateX(500px) translateY(-520px);">
    <button class="btn">F2</button>
    <button class="btn">E1</button>
  </div>
  <div class="column" style="transform: translateX(550px) translateY(-640px);">
    <button class="btn">F3</button>
    <button class="btn">E2</button>
  </div>
  <div class="column" style="transform: translateX(600px) translateY(-760px);">
    <button class="btn">F4</button>
    <button class="btn">E3</button>
  </div>
  <div class="column" style="transform: translateX(650px) translateY(-840px);">
    <button class="btn">E4</button>
  </div>
  
  <!---------------------------------------------------- ------------------------------>
 

   <div class="column" style="transform: translateX(0px) translateY(-400px);">
    
    <button class="btn">K4</button>
    
  </div>
  <div class="column" style="transform: translateX(50px) translateY(-480px);">
    
    <button class="btn">K3</button>
    <button class="btn">L4</button>
    
  </div>
  <div class="column" style="transform: translateX(100px) translateY(-600px);">
    
    <button class="btn">K2</button>
    <button class="btn">L3</button>
    
  </div>
  <div class="column" style="transform: translateX(150px) translateY(-720px);">
    
    <button class="btn">K1</button>
    <button class="btn">L2</button>
    
  </div>
  <div class="column" style="transform: translateX(200px) translateY(-800px);">
    
    <button class="btn">L1</button>
    
  </div>
  <div class="column" style="transform: translateX(300px) translateY(-760px);">
    <button class="btn">A1</button>
    <button class="btn">A2</button>
    <button class="btn">A3</button>
    <button class="btn">A4</button>
  </div>
  <div class="column" style="transform: translateX(350px) translateY(-920px);">
    <button class="btn">B1</button>
    <button class="btn">B2</button>
    <button class="btn">B3</button>
    <button class="btn">B4</button>
  </div>
   <div class="column" style="transform: translateX(450px) translateY(-1160px);">
    
    <button class="btn">C1</button>
    
  </div>
  <div class="column" style="transform: translateX(500px) translateY(-1200px);">
    
    <button class="btn">D1</button>
    <button class="btn">C2</button>
    
  </div>
  <div class="column" style="transform: translateX(550px) translateY(-1240px);">
    
    <button class="btn">D2</button>
    <button class="btn">C3</button>
    
  </div>
  <div class="column" style="transform: translateX(600px) translateY(-1280px);">
    
    <button class="btn">D3</button>
    <button class="btn">C4</button>
    
  </div>
  <div class="column" style="transform: translateX(650px) translateY(-1320px);">
    
    <button class="btn">D4</button>
    
  </div>
  <div class="column" style="transform: translateX(-25px) translateY(-1240px);">
    
    <button class="btn">leftCoralStation</button>
    
  </div>
  <div class="column" style="transform: translateX(600px) translateY(-1280px);">
    
    <button class="btn">rightCoralStation</button>
    
  </div>
  <div class="column" style="transform: translateX(-25px) translateY(-2050px);">
    
    <button class="btn">algaeInBarge</button>
    
  </div>
  <div class="column" style="transform: translateX(680px) translateY(-2100px);">
    <div class = "rotated-90">
    <button class="btn">proccesor</button>
    </div>
    
  </div>
  <div class="column" style="transform: translateX(200px) translateY(-1550px);">
    
    <button class="algae-btn">5</button>
  </div>
  <div class="column" style="transform: translateX(200px) translateY(-2000px);">
    
    <button class="algae-btn">1</button>
  </div>
  <div class="column" style="transform: translateX(425px) translateY(-1550px);">
    
    <button class="algae-btn">4</button>
  </div>
  <div class="column" style="transform: translateX(425px) translateY(-2000px);">
    
    <button class="algae-btn">2</button>
  </div>
  <div class="column" style="transform: translateX(100px) translateY(-1775px);">
    
    <button class="algae-btn">6</button>
  </div>
  <div class="column" style="transform: translateX(525px) translateY(-1775px);">
    
    <button class="algae-btn">3</button>
  </div>
  </div>
  
 
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/command', methods=['POST'])
def command():
    data = request.get_json()
    cmd = data.get('command', '').strip()

    if cmd:
        print(f"Received: {cmd}")
        nt.putBoolean(cmd, True)  # Send button name to NT as a boolean True
        # Optional: Add logic to reset it back to False after a moment
    return '', 204

def run_flask():
    app.run(host='0.0.0.0', port=5000)

if __name__ == '__main__':
    threading.Thread(target=run_flask).start()
