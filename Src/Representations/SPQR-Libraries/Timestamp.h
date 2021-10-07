/*
 * PTracking - Distributed real-time multiple object tracking library.
 * Copyright (c) 2014, Fabio Previtali. All rights reserved.
 * 
 * This file is part of PTracking.
 * 
 * PTracking is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * PTracking is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public License
 * along with PTracking. If not, see <http://www.gnu.org/licenses/>.
 * 
 * Written by Fabio Previtali.
 * 
 * Please, report any suggestion/comment/bug to fabio.previtali@gmail.com.
 */

#pragma once

#include <sys/time.h>
#include <sstream>
#include <string>

namespace Time
{
	/**
	 * @struct Timestamp
	 * 
	 * @brief Struct that represents a timestamp.
	 */
	struct Timestamp : public timeval
	{
		/**
		 * @brief Empty constructor.
		 * 
		 * It initializes the timestamp to the current time.
		 */
		Timestamp() { setToNow(); }
		
		/**
		 * @brief Constructor that takes an initialization value for the timestamp.
		 * 
		 * It initializes the timestamp with the value given as input.
		 * 
		 * @param seconds initialization value of the timestamp.
		 */
		Timestamp(float seconds) { setSeconds(seconds); }
		
		/**
		 * @brief Function that returns the milliseconds of the timestamp.
		 * 
		 * @return the milliseconds of the timestamp.
		 */
		inline float getMs() const { return (float) tv_sec * 1000 + tv_usec * 0.001; }
		
		/**
		 * @brief Function that returns the milliseconds from the midnight of the timestamp.
		 * 
		 * @return the milliseconds from the midnight of the timestamp.
		 */
		inline unsigned long getMsFromMidnight() const { return (tv_sec % 86400) * 1000 + tv_usec / 1000; }
		
		/**
		 * @brief Function that returns the seconds of the timestamp.
		 * 
		 * @return the seconds of the timestamp.
		 */
		inline float getSeconds() const { return (float) tv_sec + 0.000001 * tv_usec; }
		
		/**
		 * @brief Function that converts the timestamp into a string.
		 * 
		 * @return a string representing the timestamp.
		 */
		inline std::string getStringRepresentation() const
		{
			struct timeval tv;
			gettimeofday(&tv, 0);
			
			struct tm myTm;
			localtime_r(&tv.tv_sec, &myTm);
			
			char buffer[30];
			strftime(buffer, 30, "%Y-%m-%d-%H.%M.%S", &myTm);
			
			return std::string(buffer);
		}
		
		/**
		 * @brief Function that updates the current milliseconds of the timestamp.
		 * 
		 * @param ms new value for the timestamp.
		 */
		inline void setMs(float ms) { setSeconds(ms * 0.001); }
		
		/**
		 * @brief Function that updates the current seconds of the timestamp.
		 * 
		 * @param sec new value for the timestamp.
		 */
		inline void setSeconds(float sec) { tv_sec = (time_t) sec; tv_usec = (suseconds_t)((sec - tv_sec) * 1000000); }
		
		/**
		 * @brief Function that updates the timestamp to the current time.
		 */
		inline void setToNow() { gettimeofday(this, 0); }
		
		/**
		 * @brief Operator that computes the sum of two timestamps.
		 * 
		 * @param ts reference to the timestamp that we want to sum to the current one.
		 * 
		 * @return a new timestamp obtained by summing, for each component, the current timestamp and ts.
		 */
		inline Timestamp operator+ (const Timestamp& ts) const
		{
			Timestamp t;
			
			t.tv_sec = this->tv_sec + ts.tv_sec;
			t.tv_usec = this->tv_usec + ts.tv_usec;
			
			return t;
		}
		
		/**
		 * @brief Operator that computes the difference of two timestamps.
		 * 
		 * @param ts reference to the timestamp that we want to subtract to the current one.
		 * 
		 * @return a new timestamp obtained by subtracting, for each component, the current timestamp and ts.
		 */
		inline Timestamp operator- (const Timestamp& ts) const
		{
			Timestamp t;
			bool r = false;
			
			if (ts.tv_usec > this->tv_usec) r = true;
			
			t.tv_sec = this->tv_sec - ts.tv_sec - (r ? 1 : 0);
			t.tv_usec = this->tv_usec - ts.tv_usec + (r ? 1000000 : 0);
			
			return t;
		}
		
		/**
		 * @brief Operator that checks if the current timestamp is less than the one given as input.
		 * 
		 * The comparison of the two timestamps begins by first checking the seconds and, if equals, then by checking the microseconds.
		 * 
		 * @param ts reference to the timestamp that we want to check.
		 * 
		 * @return \b true if the current timestamp is less than ts, \b false otherwise.
		 */
		inline bool operator< (const Timestamp& ts) const { return (tv_sec < ts.tv_sec || (tv_sec == ts.tv_sec && tv_usec < ts.tv_usec)); }
		
		/**
		 * @brief Operator that checks if the current timestamp is greater than the one given as input.
		 * 
		 * The comparison of the two timestamps begins by first checking the seconds and, if equals, then by checking the microseconds.
		 * 
		 * @param ts reference to the timestamp that we want to check.
		 * 
		 * @return \b true if the current timestamp is greater than ts, \b false otherwise.
		 */
		inline bool operator> (const Timestamp& ts) const { return (tv_sec > ts.tv_sec || (tv_sec == ts.tv_sec && tv_usec > ts.tv_usec)); }
		
		/**
		 * @brief Operator that defines an implicit conversion for an object of the class to a float variable.
		 */
		inline operator float() const { return getSeconds(); }
	};
}
