#ifndef __SUBSYSTEM_H__
#define __SUBSYSTEM_H__

/*
  AeroQuad v1.5 - Novmeber 2009
 www.AeroQuad.info
 Copyright (c) 2009 Chris Whiteford.  All rights reserved.
 An Open Source Arduino based quadrocopter.
 
 This program is free software: you can redistribute it and/or modify 
 it under the terms of the GNU General Public License as published by 
 the Free Software Foundation, either version 3 of the License, or 
 (at your option) any later version. 
 
 This program is distributed in the hope that it will be useful, 
 but WITHOUT ANY WARRANTY; without even the implied warranty of 
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
 GNU General Public License for more details. 
 
 You should have received a copy of the GNU General Public License 
 along with this program. If not, see <http://www.gnu.org/licenses/>. 
 */

class SubSystem
{
	private:
		unsigned int _enabled;
		unsigned long _lastRunTime;

		unsigned int _frequency;

	public:
		SubSystem() 
		{
			_enabled = 0;
			_lastRunTime = 0;
			_frequency = 0;
		}

		void _initialize(unsigned int frequency, unsigned int offset = 0)
		{
			_frequency = frequency;
			_lastRunTime = offset;
			_enabled = 1;
		}

  		virtual void initialize(unsigned int frequency, unsigned int offset) 
		{ 
    		this->_initialize(frequency, offset); 
  		}

  		void enable()
  		{
    		_enabled = 1;
  		}

  		void disable()
  		{
    		_enabled = 0;
  		}

  		unsigned int enabled()
  		{
    		return _enabled;
  		}

  		unsigned int _canProcess(unsigned long currentTime)
  		{
			if (_enabled == 1)
			{
				if (currentTime > (_lastRunTime + _frequency))
				{
					_lastRunTime = currentTime;
					return 1;
				}
			}   
			return 0;
		}

		virtual void process(unsigned long currentTime) {
		}
};

#endif



