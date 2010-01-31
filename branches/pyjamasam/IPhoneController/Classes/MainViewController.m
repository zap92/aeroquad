//
//  MainViewController.m
//  AccelerometerTutorial
//
//  Created by Brandon Cannaday on 8/5/09.
//  Copyright 2009 Paranoid Ferret Productions. All rights reserved.
//

#import "MainViewController.h"


@implementation MainViewController

@synthesize labelX;
@synthesize labelY;
@synthesize labelZ;

@synthesize progressX;
@synthesize progressY;
@synthesize progressZ;

@synthesize accelerometer;

// Implement viewDidLoad to do additional setup after loading the view, typically from a nib.
- (void)viewDidLoad {
  [super viewDidLoad];
  
  self.accelerometer = [UIAccelerometer sharedAccelerometer];
  self.accelerometer.updateInterval = .05;
  self.accelerometer.delegate = self;
	
	_isConnected = NO;
	
	_socket = [[AsyncSocket alloc] init];
	_socket.delegate = self;
	[_socket connectToHost:@"169.254.123.219" onPort:2451 error:nil];
}

- (void)accelerometer:(UIAccelerometer *)accelerometer didAccelerate:(UIAcceleration *)acceleration 
{
  labelX.text = [NSString stringWithFormat:@"%@%f", @"X: ", acceleration.x];
  labelY.text = [NSString stringWithFormat:@"%@%f", @"Y: ", acceleration.y];
  labelZ.text = [NSString stringWithFormat:@"%@%f", @"Z: ", acceleration.z];
  
  self.progressX.progress = ABS(acceleration.x);
  self.progressY.progress = ABS(acceleration.y);
  self.progressZ.progress = ABS(acceleration.z);
	
	_pitchCommand = ((acceleration.x * 500) + 1500);
	if (_pitchCommand > 2000) { _pitchCommand = 2000; }
	if (_pitchCommand < 1000) { _pitchCommand = 1000; }
	
	_rollCommand = ((-acceleration.y * 500) + 1500);
	if (_rollCommand > 2000) { _rollCommand = 2000; }
	if (_rollCommand < 1000) { _rollCommand = 1000; }
	
	if (_isConnected == YES)
	{
		//Send a command packet to the server
		NSMutableData *packetData = [[NSMutableData alloc] init];
		[packetData appendData:[NSData dataWithBytes:&_pitchCommand length:sizeof(_pitchCommand)]];
		[packetData appendData:[NSData dataWithBytes:&_rollCommand length:sizeof(_rollCommand)]];
		[packetData appendData:[NSData dataWithBytes:&_throttleCommand length:sizeof(_throttleCommand)]];
		[packetData appendData:[NSData dataWithBytes:&_yawCommand length:sizeof(_yawCommand)]];
		[packetData appendData:[NSData dataWithBytes:&_modeCommand length:sizeof(_modeCommand)]];
		[packetData appendData:[NSData dataWithBytes:&_auxCommand length:sizeof(_auxCommand)]];
		[packetData appendData:[@"\n" dataUsingEncoding:NSASCIIStringEncoding]];
		
		 [_socket writeData:packetData withTimeout:1000 tag:1];
		[packetData release];
		packetData = nil;
	}
}


- (void)dealloc {	
    [super dealloc];
}

- (void)onSocket:(AsyncSocket *)sock didConnectToHost:(NSString *)host port:(UInt16)port
{
	NSLog(@"Connection Completed");
	_isConnected = YES;
}


@end
