
class ChannelSlider
{
  int _x;
  int _y;
  int _width;
  int _height;
  boolean _isHorzontal;

  int _centerValue;
  int _currentValue;
  int _minValue;
  int _maxValue;
  
  int _minValueSeen;
  int _maxValueSeen;
  int _calculatedCenter;

  ChannelSlider(int x, int y, int size, boolean horzontal, int minValue, int maxValue)
  {
    _x = x;
    _y = y;

    _isHorzontal = horzontal;

    if (_isHorzontal)
    {
      _width = size;
      _height = 30; 
    }
    else
    {
      _height = size;
      _width = 30;
    }

    _minValue = minValue;
    _maxValue = maxValue;

    _currentValue = minValue + (maxValue - minValue) / 2;
    _centerValue = _currentValue;
    _calculatedCenter = _currentValue;
    
    _minValueSeen = _currentValue;
    _maxValueSeen = _currentValue;
  }

  void display()
  {
    stroke(0);
    fill(255);
    rect(_x,_y,_width,_height);
    
    //Min Value Seen
    float drawPoint = (float)(_minValueSeen - _minValue) / (float)(_maxValue - _minValue);
    stroke(color(255,0,0));
    fill(color(255,0,0));
    rect((_x + (drawPoint * _width))-1, _y - 2, 2, _height + 4);
    
    //Max Value Seen
    drawPoint = (float)(_maxValueSeen - _minValue) / (float)(_maxValue - _minValue);
    stroke(color(0,255,0));
    fill(color(0,255,0));
    rect((_x + (drawPoint * _width))-1, _y - 2, 2, _height + 4);
    
    //calculated center
    drawPoint = (float)(_calculatedCenter - _minValue) / (float)(_maxValue - _minValue);
    stroke(color(255,255,0));
    fill(color(255,255,0));
    rect((_x + (drawPoint * _width))-1, _y - 2, 2, _height + 4);
    
    //CurrentValue
    drawPoint = (float)(_currentValue - _minValue) / (float)(_maxValue - _minValue);
    stroke(color(0,0,255));
    fill(color(0,0,255));
    rect((_x + (drawPoint * _width))-1, _y - 2, 2, _height + 4);
  }

  void setCurrentValue(int newValue)
  {
    if (newValue < _minValue || newValue > _maxValue)
    {
      //Do nothing.  Value is out of range
    }
    else
    {
      _currentValue = newValue;
      
      if (_currentValue < _minValueSeen)
      {
        _minValueSeen = _currentValue;
      }
      
      if (_currentValue > _maxValueSeen)
      {
        _maxValueSeen = _currentValue;
      }
      
      //Calculate the new center value
      _calculatedCenter = _minValueSeen + (_maxValueSeen - _minValueSeen) / 2;
    }
  }
}
