import cv2
import pytesseract
import numpy as np
import subprocess
b1 = 0


#def talker(b1):
    #    with open('number.txt', 'w') as f:
    #    f.write(b1)
    #    f.write('\n')
    #call(["python", "numpub.py", b1])
    #script = ["python2", "numpub.py", b1]    
    #process = subprocess.Popen(" ".join(script),
    #                                  shell=True,  
    #                                   env={"PYTHONPATH": "."})
   

def main():
	cap = cv2.VideoCapture(0)
	global b1

	while True:
		_,img = cap.read()

		#image preprocessing
		img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
		(thresh, img_binary) = cv2.threshold(img_grayscale, 128, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
		edges = cv2.Canny(img_grayscale,100,200)

		himg, wimg, _ = img.shape #size of image

		#to detect only digit
		#add configuration to extract character #please refer documentation for more details of oem and psm
		conf = r'--oem 3 --psm 6 outputbase digits'
		boxes = pytesseract.image_to_data(edges, config= conf) #output in the string #boxes = [level pageno blockno paragraphno lineno wordno left topwidth height config text]
		#print(pytesseract.image_to_string(img_binary))
		
		for x,b in enumerate(boxes.splitlines()): #to set the counter used 'enumarate' command
			if x!=0: #1st row is heading to avoid the titles used conditional statement
				b = b.split()
				
				if len(b)==12: #to select specific data used conditional statement
                                        x,y,w,h = int(b[6]), int(b[7]), int(b[8]), int(b[9])
		#			cv2.rectangle(img, (x,y),(x+w,y+h),(0,0,255),1)
		#			cv2.putText(img, b[11],(x,y), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 2)
                                        b1 = (b[11])
                                        if b1=="3034":
                                                with open('number.txt', 'w') as f:
                                                    f.write(b1)
                                                    f.write('\n')
                                                break
                                        else: 
                                                with open('number.txt', 'w') as f:
                                                    f.write("none")
                                                    f.write('\n')
                                                break  


		cv2.imshow('Result1', img)
		cv2.waitKey(1)

main()
