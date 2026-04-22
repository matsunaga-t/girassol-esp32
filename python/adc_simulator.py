import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

PLOT_ILLUMINANCE = False

PLOT_LENGTH = 20

def voltageDivider(resistor1, resistor2, voltage):
    return resistor1 * voltage / (resistor1 + resistor2)

class LDR:
    def __init__(self, lnConstant, power, resistor, voltage):
        self.const = lnConstant
        self.power = power
        self.voltage = voltage
        self.resistor = resistor
    
    def volt(self, lnIlluminance):
        return voltageDivider(self.resistor, np.exp(self.res(lnIlluminance)), self.voltage)
    
    def res(self, lnIlluminance):
        return self.const - lnIlluminance * self.power

class LDRNormalizer:
    def __init__(self, alpha, beta, voltage):
        self.alpha = alpha
        self.beta = beta
        self.voltage = voltage
    
    def normalize(self, inputVoltage):
        #return self.beta + np.log((self.voltage / volt - 1)) * self.alpha
        return np.log(self.beta * (3.3/inputVoltage - 1) ** self.alpha)
    
    def update(self, error, inputVoltage, learningRate=1):
        #d_alpha = self.beta * self.alpha * (self.voltage / inputVoltage - 1) ** (self.alpha - 1)
        #d_beta = (3.3/inputVoltage - 1) ** self.alpha
        d_alpha = (self.voltage / inputVoltage - 1)
        d_beta = 1 / self.beta
        
        self.alpha -= error * learningRate * d_alpha
        self.beta  -= error * learningRate * d_beta

rng = np.random.default_rng(42)

ldr1 = LDR(11.3373904862202, 0.615932450836737, 1000 * (rng.random() * .2 - .1 + 1), 3.3)
ldr2 = LDR(10.9327655047729, 0.57099877297531 , 1000 * (rng.random() * .2 - .1 + 1), 3.3)
ldr3 = LDR(11.2481036463693, 0.578058151321358, 1000 * (rng.random() * .2 - .1 + 1), 3.3)
ldr4 = LDR(10.6852316542604, 0.552624264297261, 1000 * (rng.random() * .2 - .1 + 1), 3.3)

normalizer1 = LDRNormalizer(-1.6235546587 , 1328.43239681, 3.3)
normalizer2 = LDRNormalizer(-1.75131724853, 1151.78689127, 3.3)
normalizer3 = LDRNormalizer(-1.72992976176, 1823.47229561, 3.3)
normalizer4 = LDRNormalizer(-1.80954776076, 930.315630501, 3.3)

plt.ion()  # turning interactive mode on

# preparing the data

error1 = []
error2 = []
error3 = []
error4 = []

I  = pd.Series(np.linspace(0, 14, 100))

#print(I.map(lambda x : normalizer4.normalize(ldr4.volt(x))).to_list())
#exit()

def update(illuminance):
    global error1
    global error2
    global error3
    global error4
    
    global normalizer1
    global normalizer2
    global normalizer3
    global normalizer4
    
    y1 = normalizer1.normalize(ldr1.volt(illuminance))
    y2 = normalizer2.normalize(ldr2.volt(illuminance))
    y3 = normalizer3.normalize(ldr3.volt(illuminance))
    y4 = normalizer4.normalize(ldr4.volt(illuminance))
    
    mean = np.mean([y1, y2, y3, y4])
    
    error1.append(y1 - mean)
    error1.pop(0)
    error2.append(y2 - mean)
    error2.pop(0)
    error3.append(y3 - mean)
    error3.pop(0)
    error4.append(y4 - mean)
    error4.pop(0)
    
    

for i in range(PLOT_LENGTH):
    illuminance = rng.random() * 14
    y1 = normalizer1.normalize(ldr1.volt(illuminance))
    y2 = normalizer2.normalize(ldr2.volt(illuminance))
    y3 = normalizer3.normalize(ldr3.volt(illuminance))
    y4 = normalizer4.normalize(ldr4.volt(illuminance))
    
    mean = np.mean([y1, y2, y3, y4])
    
    error1.append(mean - y1)
    error2.append(mean - y2)
    error3.append(mean - y3)
    error4.append(mean - y4)
    
x = [*range(0, PLOT_LENGTH)]

fig, ax = plt.subplots(1, 2, figsize=(12, 5))

# plotting the first frame
eGraph1 = ax[0].plot(x,error1, '-', c='tab:green')[0]
eGraph2 = ax[0].plot(x,error2, '-', c='tab:red')[0]
eGraph3 = ax[0].plot(x,error3, '-', c='tab:blue')[0]
eGraph4 = ax[0].plot(x,error4, '-', c='tab:orange')[0]

graph1 = ax[1].plot(I, I.map(lambda x : normalizer1.normalize(ldr1.volt(x))))[0]
graph2 = ax[1].plot(I, I.map(lambda x : normalizer2.normalize(ldr2.volt(x))))[0]
graph3 = ax[1].plot(I, I.map(lambda x : normalizer3.normalize(ldr3.volt(x))))[0]
graph4 = ax[1].plot(I, I.map(lambda x : normalizer4.normalize(ldr4.volt(x))))[0]

#plt.ylim(min(min(y1), min(y2), min(y3), min(y4)),max(max(y1), max(y2), max(y3), max(y4)))

ax[0].set_ylim(min(min(error1), min(error2), min(error3), min(error4)) * 1.1,max(max(error1), max(error2), max(error3), max(error4)) * 1.1)
plt.pause(1)

caliInterval = 0

# the update loop
while(True):
    # updating the data
    update(rng.random() * 14)
    
    # plotting newer graph
    eGraph1.set_ydata(error1)
    eGraph2.set_ydata(error2)
    eGraph3.set_ydata(error3)
    eGraph4.set_ydata(error4)
    
    graph1.set_ydata(I.map(lambda x : normalizer1.normalize(ldr1.volt(x))))
    graph2.set_ydata(I.map(lambda x : normalizer2.normalize(ldr2.volt(x))))
    graph3.set_ydata(I.map(lambda x : normalizer3.normalize(ldr3.volt(x))))
    graph4.set_ydata(I.map(lambda x : normalizer4.normalize(ldr4.volt(x))))
    
    #plt.ylim(min(min(y1), min(y2), min(y3), min(y4)),max(max(y1), max(y2), max(y3), max(y4)))
    ax[0].set_ylim(min(min(error1), min(error2), min(error3), min(error4)) * 1.1,max(max(error1), max(error2), max(error3), max(error4)) * 1.1)
    ax[1].set_ylim(min(normalizer1.normalize(ldr1.volt(0)), 
                       normalizer2.normalize(ldr2.volt(0)), 
                       normalizer3.normalize(ldr3.volt(0)), 
                       normalizer4.normalize(ldr4.volt(0))),
                   max(normalizer1.normalize(ldr1.volt(14)), 
                       normalizer2.normalize(ldr2.volt(14)), 
                       normalizer3.normalize(ldr3.volt(14)), 
                       normalizer4.normalize(ldr4.volt(14))))
    
    if caliInterval >= .5:
        normalizer1.update(mean - y1, y1, .001)
        normalizer2.update(mean - y2, y2, .001)
        normalizer3.update(mean - y3, y3, .001)
        normalizer4.update(mean - y4, y4, .001)
        caliInterval -= .5
    
    # calling pause function for 0.25 seconds
    plt.pause(0.1)
    caliInterval += 0.1