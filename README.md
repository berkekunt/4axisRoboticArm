Servo motorların akım kapasiteleri yüksek olduğundan minimum 5V 10A güç kaynağı kullanılması önerilir.(Sağlıklı bir çalışma için)
Raspberry Pi içine VsCode kurulumu yapıp main içerisindeki 4axisServo.py dosyasını çalıştırabilirsiniz. 
Raspberry Pi SD kart hazırlama ve işletim sistemi yükleme gibi kaynakları youtube'da bulabilirsiniz.
Raspberry Pi kamera config ayarları için komut satırını açıp;
	1 = sudo apt updgrade
	2 = sudo raspi-config
	3 = Interface Options
	4 = Legacy Camera (Enable olarak ayarlamalısınız)
	5 = reboot
Kamera testi için komut satırına;
	raspistill -o Desktop/image.jpg  (Bir fotoğraf çeker ve masaüstüne JPEG olarak kaydeder.)
	raspivid -o Desktop/video.h264  (Bir video çeker ve masaüstüne kaydeder.)

Kamera ayarları yapıldıktan sonra ihtiyacımız olan kütüphaneleri ve eklentileri yüklemeliyiz;
	1 = İlk olarak VsCODE yüklü değil ise terminale = sudo apt install code
	2 = OpenCV kütüphanesini yüklemeniz gerekiyor = sudo apt-get install libhdf5-dev -y && sudo apt-get install libhdf5-serial-dev -y && sudo apt-get install libatlas-base-dev -y && sudo apt-get install libjasper-dev -y && sudo apt-get install libqtgui4 libqtwebkit4 libqt4-test
	3 = Video ve image kütüphaneleri ; sudo apt-get install libjpeg-dev libpng-dev libtiff-dev && sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev && sudo apt-get install libxvidcore-dev libx264-dev
	4 = Numpy ve Python kurulumu ;
		sudo apt-get install python3-dev
		sudo apt-get install python-pip
		sudo apt install python3-matplotlib
		sudo apt-get install python3-numpy
	5 = Son olarak OpenCV kütüphanesini kuruyoruz; pip3 install opencv-contrib-python==4.1.0.25
PCA servo sürücü kütüphanesi için ;  pip3 install adafruit-circuitpython-servokit


Artık kodu test etmeye hazırsınız lütfen kod içindeki motor açı ayarlarını test edip kendinize göre ayarlayın :)


-----------------------------------------------------------------

For English; 

Since servo motors have high current capacities, it is recommended to use at least a 5V 10A power supply for stable operation.

You can install VSCode on the Raspberry Pi and run the 4axisServo.py file inside the main directory.

You can find resources on YouTube for preparing a Raspberry Pi SD card and installing the operating system.

For Raspberry Pi camera configuration, open the command line and run:

1= sudo apt upgrade
2 =sudo raspi-config
3 =Go to Interface Options
4 =Select Legacy Camera (Set it to Enable)
5 =Reboot
For camera testing, use the following commands in the terminal:

raspistill -o Desktop/image.jpg (Captures a photo and saves it as a JPEG on the desktop.)
raspivid -o Desktop/video.h264 (Records a video and saves it on the desktop.)
After configuring the camera, we need to install the required libraries and dependencies:

1 = If VSCode is not installed, run:
sudo apt install code

2 = Install the OpenCV library:
sudo apt-get install libhdf5-dev -y && sudo apt-get install libhdf5-serial-dev -y && sudo apt-get install libatlas-base-dev -y && sudo apt-get install libjasper-dev -y && sudo apt-get install libqtgui4 libqtwebkit4 libqt4-test

3 = Install video and image processing libraries:
sudo apt-get install libjpeg-dev libpng-dev libtiff-dev && sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev && sudo apt-get install libxvidcore-dev libx264-dev

4 = Install NumPy and Python dependencies:
sudo apt-get install python3-dev
sudo apt-get install python-pip
sudo apt install python3-matplotlib
sudo apt-get install python3-numpy

5 = Finally, install the OpenCV package:
pip3 install opencv-contrib-python==4.1.0.25

For the PCA servo driver library, install:
pip3 install adafruit-circuitpython-servokit

Now you are ready to test the code! Please test the motor angle settings inside the code and adjust them according to your needs.
