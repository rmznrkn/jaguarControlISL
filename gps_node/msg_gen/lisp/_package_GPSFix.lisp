(cl:in-package gps_node-msg)
(cl:export '(HEADER-VAL
          HEADER
          STATUS-VAL
          STATUS
          LATITUDE-VAL
          LATITUDE
          LONGITUDE-VAL
          LONGITUDE
          ALTITUDE-VAL
          ALTITUDE
          TRACK-VAL
          TRACK
          SPEED-VAL
          SPEED
          CLIMB-VAL
          CLIMB
          PITCH-VAL
          PITCH
          ROLL-VAL
          ROLL
          DIP-VAL
          DIP
          TIME-VAL
          TIME
          GDOP-VAL
          GDOP
          PDOP-VAL
          PDOP
          HDOP-VAL
          HDOP
          VDOP-VAL
          VDOP
          TDOP-VAL
          TDOP
          ERR-VAL
          ERR
          ERR_HORZ-VAL
          ERR_HORZ
          ERR_VERT-VAL
          ERR_VERT
          ERR_TRACK-VAL
          ERR_TRACK
          ERR_SPEED-VAL
          ERR_SPEED
          ERR_CLIMB-VAL
          ERR_CLIMB
          ERR_TIME-VAL
          ERR_TIME
          ERR_PITCH-VAL
          ERR_PITCH
          ERR_ROLL-VAL
          ERR_ROLL
          ERR_DIP-VAL
          ERR_DIP
          POSITION_COVARIANCE-VAL
          POSITION_COVARIANCE
          POSITION_COVARIANCE_TYPE-VAL
          POSITION_COVARIANCE_TYPE
          LASTRAWDATA-VAL
          LASTRAWDATA
          RMC1UTCTIME-VAL
          RMC1UTCTIME
          RMC2STATUS-VAL
          RMC2STATUS
          RMC3LATITUDE-VAL
          RMC3LATITUDE
          RMC4LATNORS-VAL
          RMC4LATNORS
          RMC5LONGITUDE-VAL
          RMC5LONGITUDE
          RMC6LONGEORW-VAL
          RMC6LONGEORW
          RMC7SPEEDOVERGROUNDINKNOTS-VAL
          RMC7SPEEDOVERGROUNDINKNOTS
          RMC8TRACKMADEGOODDEGREETRUE-VAL
          RMC8TRACKMADEGOODDEGREETRUE
          RMC9DATE-VAL
          RMC9DATE
          RMC10MAGNETICVARIATIONDEGREES-VAL
          RMC10MAGNETICVARIATIONDEGREES
          RMC11MAGNETEORW-VAL
          RMC11MAGNETEORW
          RMC12FAAMODEINDICATOR-VAL
          RMC12FAAMODEINDICATOR
          GGA1UTC-VAL
          GGA1UTC
          GGA2LATITUDE-VAL
          GGA2LATITUDE
          GGA3LATNORS-VAL
          GGA3LATNORS
          GGA4LONGITUDE-VAL
          GGA4LONGITUDE
          GGA5LONGEORW-VAL
          GGA5LONGEORW
          GGA6GPSQUALITY-VAL
          GGA6GPSQUALITY
          GGA6GPSQUALITYSTATUS-VAL
          GGA6GPSQUALITYSTATUS
          GGA7NUMBEROFSATELLITES-VAL
          GGA7NUMBEROFSATELLITES
          GGA8HORIZONTALDILUTIONOFPRESCISION-VAL
          GGA8HORIZONTALDILUTIONOFPRESCISION
          GGA9ANTENNAALTITUDEABOVEBELOWMEANSEALEVEL-VAL
          GGA9ANTENNAALTITUDEABOVEBELOWMEANSEALEVEL
          GGA10UNITSOFANTENNAALTITUDE_INMETERS-VAL
          GGA10UNITSOFANTENNAALTITUDE_INMETERS
          GGA11GEOIDALSEPARATION-VAL
          GGA11GEOIDALSEPARATION
          GGA12UNITSOFGEOIDALSEPERATION-VAL
          GGA12UNITSOFGEOIDALSEPERATION
          GGA13AGEOFDIFFERENTIALGPSDATA-VAL
          GGA13AGEOFDIFFERENTIALGPSDATA
          GGA14DIFFERENTIALREFERENCESTATIONID-VAL
          GGA14DIFFERENTIALREFERENCESTATIONID
          GSA1SELECTIONMODE-VAL
          GSA1SELECTIONMODE
          GSA2MODE-VAL
          GSA2MODE
          GSA3ID1-VAL
          GSA3ID1
          GSA4ID2-VAL
          GSA4ID2
          GSA5ID3-VAL
          GSA5ID3
          GSA6ID4-VAL
          GSA6ID4
          GSA7ID5-VAL
          GSA7ID5
          GSA8ID6-VAL
          GSA8ID6
          GSA9ID7-VAL
          GSA9ID7
          GSA10ID8-VAL
          GSA10ID8
          GSA11ID9-VAL
          GSA11ID9
          GSA12ID10-VAL
          GSA12ID10
          GSA13ID11-VAL
          GSA13ID11
          GSA14ID12-VAL
          GSA14ID12
          GSA15PDOP-VAL
          GSA15PDOP
          GSA16HDOP-VAL
          GSA16HDOP
          GSA17VDOP-VAL
          GSA17VDOP
          GSVADET-VAL
          GSVADET
          GSV1NUMBEROFGSV-VAL
          GSV1NUMBEROFGSV
          GSV2NUMBEROFMESSAGE-VAL
          GSV2NUMBEROFMESSAGE
          GSV3NUMBEROFSTALLITESINVIEW-VAL
          GSV3NUMBEROFSTALLITESINVIEW
          GSV4SATELLITEPRNNUMBER-VAL
          GSV4SATELLITEPRNNUMBER
          GSV5ELEVATIONDEGREES-VAL
          GSV5ELEVATIONDEGREES
          GSV6AZIMUTHINDEGRIES-VAL
          GSV6AZIMUTHINDEGRIES
          GSV7SRNIN_DB-VAL
          GSV7SRNIN_DB
          GSV8MORESATELLITEINFOQUADRUPLES-VAL
          GSV8MORESATELLITEINFOQUADRUPLES
))