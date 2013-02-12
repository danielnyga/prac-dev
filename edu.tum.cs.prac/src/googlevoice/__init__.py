from recognize import Voice
from microphone import MicLevelController


if __name__ == '__main__':
#    wavFileName = '/home/nyga/tmp/speech.wav'
#    flacFileName = '/home/nyga/tmp/speech.flac'
#    
#    wavFile = open(wavFileName, 'w')
#    process = Popen(['/usr/bin/arecord', '-f', 'S16_LE', '-t', 'wav', '-d', '5', '-c', '1', '-r', '16000'], stdout=wavFile)
#    time.sleep(7)
#    process.kill()
#    wavFile.close()
#    process = Popen(['/usr/bin/ffmpeg', '-i', wavFileName, '-y', flacFileName])
#    process.wait()
#    
#    url = "https://www.google.com/speech-api/v1/recognize?xjerr=1&client=chromium&lang=en-US"
#    flac = open(flacFileName,'r').read()
#    header = {'Content-Type' : 'audio/x-flac; rate=16000'}
#    req = urllib2.Request(url, flac, header)
#    data = urllib2.urlopen(req)
#    print data.read()
    voice = Voice()
    
    def analyze():
        voice.stopRecording()
        print voice.analyze()
        
    mic = MicLevelController(sensitivity=900)
    mic.registerStartCallback(voice.startRecording)
    mic.registerEndCallback(analyze)
    mic.listen()
    
    while True:
        pass