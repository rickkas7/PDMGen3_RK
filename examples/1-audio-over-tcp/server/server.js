// Install
// npm install
// 
// Run
// npm run

var fs = require('fs');
var path = require('path');
var net = require('net');
var wav = require('wav'); // https://github.com/TooTallNate/node-wav

// For this simple test, just create wav files in the "out" directory in the directory
// where audioserver.js lives.
var outputDir = path.join(__dirname, "out");  

var dataPort = 7123; // this is the port to listen on for data from the Photon

// If changing the sample frequency in the Particle code, make sure you change this!
var wavOpts = {
	'channels':1,
	'sampleRate':16000,
	'bitDepth':8
};

// Output files in the out directory are of the form 00001.wav. lastNum is used 
// to speed up scanning for the next unique file.
var lastNum = 0;

// Create the out directory if it does not exist
try {
	fs.mkdirSync(outputDir);
}
catch(e) {
}

// Start a TCP Server. This is what receives data from the Particle Photon
// https://gist.github.com/creationix/707146
net.createServer(function (socket) {
	console.log('data connection started from ' + socket.remoteAddress);
	
	// The server sends a 8-bit byte value for each sample. Javascript doesn't really like
	// binary values, so we use setEncoding to read each byte of a data as 2 hex digits instead.
	socket.setEncoding('hex');
	
	var outPath = getUniqueOutputPath();
	
	var writer = new wav.FileWriter(outPath, wavOpts);
	
	socket.on('data', function (data) {
		// We received data on this connection.
		// var buf = Buffer.from(data, 'hex');
		var buf = new Buffer(data, 'hex');
		
		if (wavOpts.bitDepth == 16) {
			// The Photon sends up unsigned data for both 8 and 16 bit
			// The wav file format is unsigned for 8 bit and signed two-complement for 16-bit. Go figure.
			for(var ii = 0; ii < buf.length; ii += 2) {
				var unsigned = buf.readUInt16LE(ii);
				var signed = unsigned - 32768;
				buf.writeInt16LE(signed, ii);
			}
		}
		
		// console.log("got data " + (data.length / 2));
		writer.write(buf);
	});
	socket.on('end', function () {
		console.log('transmission complete, saved to ' + outPath);
		writer.end();
	});
}).listen(dataPort);


function formatName(num) {
	var s = num.toString();
	
	while(s.length < 5) {
		s = '0' + s;
	}
	return s + '.wav';
}

function getUniqueOutputPath() {
	for(var ii = lastNum + 1; ii < 99999; ii++) {
		var outPath = path.join(outputDir, formatName(ii));
		try {
			fs.statSync(outPath);
		}
		catch(e) {
			// File does not exist, use this one
			lastNum = ii;
			return outPath;
		}
	}
	lastNum = 0;
	return "00000.wav";
}

