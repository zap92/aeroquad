//
//  MainViewController.h
//  AccelerometerTutorial
//
//  Created by Brandon Cannaday on 8/5/09.
//  Copyright 2009 Paranoid Ferret Productions. All rights reserved.
//

#import <UIKit/UIKit.h>
#import "AsyncSocket.h"


@interface MainViewController : UIViewController <UIAccelerometerDelegate> {
  IBOutlet UILabel *labelX;
  IBOutlet UILabel *labelY;
  IBOutlet UILabel *labelZ;
  
  IBOutlet UIProgressView *progressX;
  IBOutlet UIProgressView *progressY;
  IBOutlet UIProgressView *progressZ;
  
  UIAccelerometer *accelerometer;
	
	AsyncSocket *_socket;
	
	int _pitchCommand;
	int _rollCommand;
	int _yawCommand;
	int _throttleCommand;
	int _modeCommand;
	int _auxCommand;
	
	int _packetStart;
	BOOL _isConnected;
}

@property (nonatomic, retain) IBOutlet UILabel *labelX;
@property (nonatomic, retain) IBOutlet UILabel *labelY;
@property (nonatomic, retain) IBOutlet UILabel *labelZ;

@property (nonatomic, retain) IBOutlet UIProgressView *progressX;
@property (nonatomic, retain) IBOutlet UIProgressView *progressY;
@property (nonatomic, retain) IBOutlet UIProgressView *progressZ;

@property (nonatomic, retain) UIAccelerometer *accelerometer;

@end
