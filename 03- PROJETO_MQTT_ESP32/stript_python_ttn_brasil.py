import requests
import json
import csv
import numpy as np
from pandas import DataFrame
from pyproj import Proj


myProj = Proj("+proj=utm +zone=23 +south +ellps=GRS80 +towgs84=0,0,0,0,0,0,0 +units=m +no_defs")

url = 'https://www.thethingsnetwork.org/gateway-data/country/br'
resp = requests.get(url=url)
vetor = resp.json()
data = [ vetor ]

count = 0

with open('players.csv', 'w', newline='') as file:
    fieldnames = [ 'id', 'longiture', 'latitude', 'altitude']
    writer = csv.DictWriter(file, fieldnames=fieldnames)
    writer.writeheader()

    print(len(data) )
    for x in range( 0, len(data) ):
      for t, y in data[x].items():
        try:
            print( y['location']['latitude'] )
            print( y['location']['longitude'] )
            print( y['location']['altitude'] )
            count = count + 1
            utmlong, utmlat = myProj(y['location']['longitude'], y['location']['latitude'])
            writer.writerow({'id': count, 'longiture': int(utmlong), 'latitude': int(utmlat), 'altitude': y['location']['altitude'] })
        except:
            print("An exception occurred")


