FROM python:3.10
COPY ./pyqtViz ./pyqtViz
COPY ./src ./src
ADD main_viz.py .

RUN pip install pandas
RUN pip install multical
RUN pip install plotly
RUN pip install scikit-learn
RUN pip install openpyxl
RUN pip install numpy==1.23.4
RUN pip install matplotlib==3.1.0
RUN pip install PyQt5
RUN pip install opencv-contrib-python-headless==4.6.0.66
RUN apt-get update && apt-get install ffmpeg libsm6 libxext6  -y

CMD ["python", "./main_viz.py"]