#!/usr/bin/env python

from ublox_m8 import UbloxM8Python
import time

def twoDigitHex( number ):
  return '%02x' % number

repeats = 20


def rawx(meas, timestamp, recv_id):
  global repeats
  print ''
  print 'rawx callback {0}'.format(repeats)
  print recv_id
  for i in xrange(meas.numMeas):
    print 'psr {0} cp {1}'.format(meas.repeated_block[i].prMes,
      meas.repeated_block[i].cpMes)
  repeats = repeats - 1


def sfrbx(sfrb, timestamp, recv_id):
  print ''
  print 'sfrbx callback {0}'.format(repeats)
  print recv_id
  for i in xrange(sfrb.numWords):
    print 'navword: {0}'.format(hex(sfrb.dwrds[i]))


def posllh(pos, timestamp, recv_id):
  print ''
  print 'posllh callback'
  print "crc {0}{1}".format(twoDigitHex(pos.checksum[0]),
                            twoDigitHex(pos.checksum[1]))


def posecef(pos, timestamp, recv_id):
  print ''
  print 'posecef callback'
  print "xyz {0} {1} {2}".format(pos.ecefX, pos.ecefY, pos.ecefZ)


def velned(vel, timestamp, recv_id):
  print ''
  print 'velned callback'
  print "{0} {1} {2}".format(vel.velocity_north,
                             vel.velocity_east,
                             vel.velocity_down)


def velecef(vel, timestamp, recv_id):
  print ''
  print 'velecef callback'
  print "{0} {1} {2} {0}".format(vel.ecefVX,
                             vel.ecefVY,
                             vel.ecefVZ,
                             vel.sAcc)
  print "crc {0}{1}".format(twoDigitHex(vel.checksum[0]),
                            twoDigitHex(vel.checksum[1]))


def svinfo(info, timestamp, recv_id):
  print ''
  print 'svinfo callback {0}'.format(repeats)
  print recv_id
  for i in xrange(info.numch):
    print 'svid: {0}; elev: {0}; azim: {0}'.format(info.svinfo_repeated[i].svid, info.svinfo_repeated[i].elev, info.svinfo_repeated[i].azim)


ubx = UbloxM8Python("/dev/ttyACM0")
ubx.Connect("/dev/ttyACM0", 115200)

ubx.python_set_rxm_rawx_callback(rawx)
ubx.SetCfgMsgRate(0x02,0x15,1)

ubx.python_set_rxm_sfrbx_callback(sfrbx)
ubx.SetCfgMsgRate(0x02,0x13,1)

ubx.python_set_nav_posllh_callback(posllh)
ubx.SetCfgMsgRate(0x01,0x02,1)

ubx.python_set_nav_posecef_callback(posecef)
ubx.SetCfgMsgRate(0x01,0x01,1)

ubx.python_set_nav_velned_callback(velned)
ubx.SetCfgMsgRate(0x01,0x12,1)

ubx.python_set_nav_velecef_callback(velecef)
ubx.SetCfgMsgRate(0x01,0x11,1)

ubx.python_set_nav_svinfo_callback(svinfo)
ubx.SetCfgMsgRate(0x01,0x30,1)


while repeats:
  time.sleep(0.1)

ubx.Disconnect()
time.sleep(1)
del ubx
time.sleep(1)

