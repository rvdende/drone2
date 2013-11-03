var siteconfig = {
	name: 'viewer',
	domain: undefined,
	port: 80,
	version: "0.0.1",
}

var sys = require("sys")



///////
//  ARDUINO SERIAL
/////
var sp = require("serialport")
var arduino;

sp.list(function (err, ports) {
	/* This lists all the serial devices.. we are looking for the arduino */
	console.log("\ncomName\t\tmanufacturer")
  	console.log("====================================================================")
  	console.log(ports)
	arduino = new sp.SerialPort("COM9", { baudrate: 115200, parser: sp.parsers.readline()  } ); 	
  	arduino.on("open", arduinoConnect);

  	/*
  	ports.forEach(function(port) {
  		
  		if (port.manufacturer != undefined) {
  			//WINDOWS
  			console.log(port.comName+"\t\t"+port.manufacturer);
	  		var n=port.manufacturer.indexOf("arduino");
	  		if (n >= 0) {
	 			arduino = new sp.SerialPort(port.comName, { 
	 				baudrate: 115200, parser: sp.parsers.readline()  } ); 				
	 			arduino.on("open", arduinoConnect);
	  		}
  		} else {
  			//console.log(port)
  			console.log(port.comName+"\t\t"+port.pnpId);
  			var n=port.pnpId.indexOf("Arduino");
	  		if (n >= 0) {
	 			arduino = new sp.SerialPort(port.comName, { 
	 				baudrate: 115200, parser: sp.parsers.readline()  } ); 				
	 			arduino.on("open", arduinoConnect);
	  		}
  		}
  		
  		
    	
  	});
	*/
  	console.log("====================================================================")
});

function arduinoConnect() {
	console.log("open - connected to arduino\n\n")
	arduino.on("data", dataParser);


}



function dataParser(data) {
	var indata = "";
	try {	
	    indata = JSON.parse(data.toString()); 	
	    if (indata.status) { console.log(indata); return; }		
    	process.stdout.clearLine();
    	process.stdout.cursorTo(0);
    	process.stdout.write((1 / indata.t).toFixed(2) + "hz\t")	
    	//process.stdout.write(JSON.stringify(indata))	
	    io.sockets.emit("locator", indata)	   

	  } catch (er) {
	  	process.stdout.clearLine();
	    process.stdout.cursorTo(0);
	    process.stdout.write('invalid JSON data from arduino cannot parse.');
	    return;
	  }
      

      
  


	
	/*if ((data[1] == 13) && (data[2] == 10)) {
		
	}*/
	
	
	//sengyrovecx = (int16_t)(xhg << 8 | xlg);
   //sengyrovecy = (int16_t)(yhg << 8 | ylg);
   //sengyrovecz = (int16_t)(zhg << 8 | zlg);
   //var a = data[1] << 8 | data[0]
	//console.log(a)
	/*

	//console.log(data.toString());	
	//convert datastream to string, then parse to JSON object

	var indata;
	try {	//try to parse the arduino serial data, catch errors
	    indata = JSON.parse(data.toString()); 		
	  } catch (er) {
	    console.log('error', new Error('invalid JSON data from arduino cannot parse.'));
	    return;
	  }

	//var indata = JSON.parse(data.toString()); 		
	//console.log(data.toString());	
	if (indata.status) {
		console.log("Status:"+indata.status);	
	}
	if (testdata == 0) {
		//only print data once.
		//console.log(data.toString());
		console.log('\n\nGetting good data feed...\n')
		console.log(indata);
		//console.log(indata.motors[0])
		testdata = 1;
	}

	io.sockets.emit("locator", indata)	
	*/

}

///////
//  WEBSERVER
/////

var http 		= require('http')				//HTTP
var	url	 		= require('url')
var path 		= require('path')
var fs 			= require('fs')					//FILESYSTEM				
var util 		= require('util')

var mimeTypes = {
    "html": "text/html",
    "jpeg": "image/jpeg",
    "jpg": "image/jpeg",
    "png": "image/png",
    "js": "text/javascript",
    "css": "text/css"};

var arrow = http.createServer(function (req, res) {
	var uri = url.parse(req.url).pathname;
	var filename = path.join(process.cwd()+"/static", uri);
	//console.log(uri)
	if (uri == "/") {
			var page = path.join(process.cwd()+"/static", "/index.htm");			
			var mimeType = mimeTypes[path.extname(page).split(".")[1]]
			res.writeHead(200, {'Content-Type': mimeType})
			var fileStream = fs.createReadStream(page);
			fileStream.pipe(res);
			return;
	}
	

	//####################   STATIC FILES ################
	fs.exists(filename, function(exists) {
		if (!exists) {
			//console.log("404:"+filename);		
			res.writeHead(404, {'Content-Type': 'text/html'});
		  	res.write('error 404');
		  	res.end();
		  	return;
		} 

		var stats = fs.statSync(filename);
		//console.log("STATS: "+stats.isDirectory());

		if (stats.isDirectory()) {
			res.writeHead(200, {'Content-Type':'text/html'})
			res.write('Are you lost?')
			res.end();
		} else {
			var mimeType = mimeTypes[path.extname(filename).split(".")[1]]
			res.writeHead(200, {'Content-Type': mimeType})
			var fileStream = fs.createReadStream(filename);
			fileStream.pipe(res);	
		}		
	});	//end static server
	
})

var io = require('socket.io').listen(arrow);
io.set('log level', 1);

io.sockets.on('connection', function (socket) {
	console.log("client connected")
  socket.emit('news', { text:'socket.io', done:false });
  socket.on('my other event', function (data) {
    console.log(data);
    socket.broadcast.emit('incoming',{ textinc: data.my});
  });
  socket.on('todo', function (data) {
    console.log(data);
  });  

  socket.on('key', function (data) {

  	console.log(data)
  	var lights = {
  		cmd: "G1",
  		x: 0.1,
  		y: 0.1
  	}
  	lights.x = data.data-1;  	
  	lights.x = lights.x.toString();
  	lights.y = lights.y.toString();
  	console.log()
  	//arduino.write(JSON.stringify(lights))
  	//arduino.write('{"cmd":"G1","x": "0.5", "y":"0.1"}');

  })

  socket.on('balance', function (data) {
  	console.log(data)
  	var command = {
  		cmd: "G1",
  		x: 0
  	}

  	command.x = data.data.toString();  	
  	//arduino.write(JSON.stringify(command))
  	//arduino.write('{"cmd":"G1","x": "0.5", "y":"0.1"}');
  })

socket.on('drone', function (data) {
  	console.log("sending to drone:" +JSON.stringify(data))
  	arduino.write(JSON.stringify(data))
  })

socket.on('motors', function (data) {
  	console.log(data)
  	var command = {
  		cmd: "G1",
  		m0: 0,
  		m1: 0,
  		m2: 0,
  		m3: 0
  	}

  	command.m0 = data.m0.toString();  	
  	//command.m1 = data.m1.toString();  	
  	command.m2 = data.m2.toString();  	
  	//command.m3 = data.m3.toString();  	
  	
  	console.log("sending to drone:" + JSON.stringify(command))
  	arduino.write(JSON.stringify(command))
  	//arduino.write('{"cmd":"G1","x": "0.5", "y":"0.1"}');
  })

});

//START IT UP
console.log("Starting up server")
arrow.listen(siteconfig.port, siteconfig.domain);
if (siteconfig.domain == undefined) {
	console.log("Open your browser on localhost:"+siteconfig.port)
} else {
	console.log("Open your browser on " + siteconfig.domain + ":"+siteconfig.port)
}
