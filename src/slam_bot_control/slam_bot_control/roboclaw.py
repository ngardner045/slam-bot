#!/usr/bin/env python3

import serial
import struct
import time
import math


class RoboClaw:
    """Interface for RoboClaw 2x15A motor controller"""
    
    def __init__(self, port='/dev/ttyUSB0', baud_rate=115200, timeout=1):
        """Initialize RoboClaw connection"""
        self.serial = serial.Serial(port, baud_rate, timeout=timeout)
        self.address = 128  # Default address
        
    def close(self):
        """Close serial connection"""
        if self.serial.is_open:
            self.serial.close()
    
    def _crc16(self, data):
        """Calculate CRC16 checksum"""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc &= 0xFFFF
        return crc
    
    def _send_command(self, address, command, data=None):
        """Send command to RoboClaw"""
        if data is None:
            data = []
        
        # Build packet
        packet = [address, command] + data
        crc = self._crc16(packet)
        packet.extend([crc >> 8, crc & 0xFF])
        
        # Send packet
        self.serial.write(bytes(packet))
        time.sleep(0.01)  # Small delay for processing
        
        # Read response
        if self.serial.in_waiting > 0:
            response = self.serial.read(self.serial.in_waiting)
            return response
        return None
    
    def set_motor_velocity(self, motor, velocity):
        """Set motor velocity (-1.0 to 1.0)"""
        # Convert to RoboClaw units (-127 to 127)
        velocity_int = int(velocity * 127)
        velocity_int = max(-127, min(127, velocity_int))
        
        if motor == 1:  # Left motor
            command = 0x00  # M1 Forward
            if velocity_int < 0:
                command = 0x01  # M1 Backward
                velocity_int = abs(velocity_int)
        elif motor == 2:  # Right motor
            command = 0x04  # M2 Forward
            if velocity_int < 0:
                command = 0x05  # M2 Backward
                velocity_int = abs(velocity_int)
        else:
            raise ValueError("Motor must be 1 or 2")
        
        self._send_command(self.address, command, [velocity_int])
    
    def set_motor_velocity_differential(self, left_velocity, right_velocity):
        """Set both motors with differential drive velocities"""
        self.set_motor_velocity(1, left_velocity)
        self.set_motor_velocity(2, right_velocity)
    
    def get_encoder_count(self, motor):
        """Get encoder count for specified motor"""
        if motor == 1:
            command = 0x10  # Read M1 encoder
        elif motor == 2:
            command = 0x11  # Read M2 encoder
        else:
            raise ValueError("Motor must be 1 or 2")
        
        response = self._send_command(self.address, command)
        if response and len(response) >= 6:
            # Parse 32-bit signed integer
            count = struct.unpack('>i', response[1:5])[0]
            return count
        return 0
    
    def reset_encoder_count(self, motor):
        """Reset encoder count for specified motor"""
        if motor == 1:
            command = 0x12  # Reset M1 encoder
        elif motor == 2:
            command = 0x13  # Reset M2 encoder
        else:
            raise ValueError("Motor must be 1 or 2")
        
        self._send_command(self.address, command)
    
    def get_motor_current(self, motor):
        """Get motor current in amps"""
        if motor == 1:
            command = 0x14  # Read M1 current
        elif motor == 2:
            command = 0x15  # Read M2 current
        else:
            raise ValueError("Motor must be 1 or 2")
        
        response = self._send_command(self.address, command)
        if response and len(response) >= 3:
            # Parse 16-bit unsigned integer
            current = struct.unpack('>H', response[1:3])[0]
            return current / 100.0  # Convert to amps
        return 0.0
    
    def get_battery_voltage(self):
        """Get battery voltage"""
        command = 0x24  # Read main battery voltage
        response = self._send_command(self.address, command)
        if response and len(response) >= 3:
            # Parse 16-bit unsigned integer
            voltage = struct.unpack('>H', response[1:3])[0]
            return voltage / 10.0  # Convert to volts
        return 0.0
    
    def set_velocity_pid_constants(self, motor, p, i, d, qpps):
        """Set PID constants for velocity control"""
        if motor == 1:
            command = 0x28  # Set M1 velocity PID
        elif motor == 2:
            command = 0x29  # Set M2 velocity PID
        else:
            raise ValueError("Motor must be 1 or 2")
        
        # Convert to integers
        p_int = int(p * 65536)
        i_int = int(i * 65536)
        d_int = int(d * 65536)
        qpps_int = int(qpps)
        
        data = [
            (p_int >> 24) & 0xFF,
            (p_int >> 16) & 0xFF,
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (i_int >> 24) & 0xFF,
            (i_int >> 16) & 0xFF,
            (i_int >> 8) & 0xFF,
            i_int & 0xFF,
            (d_int >> 24) & 0xFF,
            (d_int >> 16) & 0xFF,
            (d_int >> 8) & 0xFF,
            d_int & 0xFF,
            (qpps_int >> 24) & 0xFF,
            (qpps_int >> 16) & 0xFF,
            (qpps_int >> 8) & 0xFF,
            qpps_int & 0xFF
        ]
        
        self._send_command(self.address, command, data)
    
    def emergency_stop(self):
        """Emergency stop - stop all motors"""
        self._send_command(self.address, 0x06)  # Stop all motors
