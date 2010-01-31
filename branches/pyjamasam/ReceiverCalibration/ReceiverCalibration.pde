import processing.opengl.*;
import processing.serial.*;

Serial myPort;
ChannelSlider[] channels;

boolean dataFlowing = false;

void setup()
{
  size(320,340);

  println (Serial.list());

  myPort = new Serial(this, Serial.list()[0], 115200);
  myPort.bufferUntil('\n');

  channels = new ChannelSlider[6];

  channels[0] = new ChannelSlider(10,10,300, true, 1000, 2000);
  channels[1] = new ChannelSlider(10,50,300, true, 1000, 2000);

  channels[2] = new ChannelSlider(10,105,300, true, 1000, 2000);
  channels[3] = new ChannelSlider(10,145,300, true, 1000, 2000);

  channels[4] = new ChannelSlider(10,200,300, true, 1000, 2000);
  channels[5] = new ChannelSlider(10,240,300, true, 1000, 2000);
}

void draw()
{
  if (!dataFlowing)
  {
    println("Waiting for the arduino to be ready after opening the port...");
    delay(2000);
    myPort.write('d');

    dataFlowing = true;
  }
  
  //update(mouseX, mouseY);
  background(255,255,255);

  for(int i = 0; i < 6; i++)
  {
    channels[i].display();
  }
}

byte[] inBuffer = new byte[200];
void serialEvent (Serial myPort) 
{
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) 
  {
    // trim off any whitespace:
    inString = trim(inString);

    String[] splitValues = inString.split(",");
    for (int i = 0; i < splitValues.length; i++)
    {
      if (i < channels.length)
      {
        int inValue = int(splitValues[i]);
        channels[i].setCurrentValue(inValue);
      }
    }
  }
}













