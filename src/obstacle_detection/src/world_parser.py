import numpy as np
import xml.etree.ElementTree as ET

class WorldParser:

    def __init__(self,infile):
        self.parser = ET.parse(infile)

    def allZonotopes(self):
        cylinders = self.getCylinders()
        zonotopes = np.array([self.cyl2zon(c) for c in cylinders])

        return zonotopes

    def getCylinders(self):

        cylinders = []
        root = self.parser.getroot()
        for m in root.findall('./world/model'):
            c = m.find('./link/collision/geometry/cylinder')
            if not c:
                continue

            cyl = {}
            for x in list(c):
                if x.tag == 'length':
                    cyl['length'] = float(x.text)
                elif x.tag == 'radius':
                    cyl['radius'] = float(x.text)
            
            cyl['pose'] = [float(x) for x in m.find('pose').text.split()]
            cylinders.append(cyl)

        return cylinders

    def cyl2zon(self,cyl,num_sides=8):
        '''
        Take a cylinder dict (length,radius,pose keys) and return a zonotope
        matrix
        '''
        radius = cyl['radius']
        center = cyl['pose'][:3]
        length = cyl['length']

        scale = 0.3252*num_sides - 0.214 # Magic Number
        dtheta = 2*np.pi/num_sides
        G = np.zeros((3,num_sides//2+2))
        G[:,0] = center
        G[:,-1] = [0,0,length/2]

        for i in range(num_sides//2):
            theta = (i-1)*dtheta
            G[:,i+1] = [radius*np.cos(theta)/scale,radius*np.sin(theta)/scale,0]

        return G
