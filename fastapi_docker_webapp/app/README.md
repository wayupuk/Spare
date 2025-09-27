# Main Programs

```
/predict-hand
use in api calling to do a hand prediction from row-by-row data
```


```
/predict_csv
use in api calling to do a hand prediction using csv data
```


```
/upload-audio
Use in ASR
```


we use Websocket to send output back to wep application

we use fastapi file upload to receive uploaded file from wep application. the fast api will call a static folder to load files in static e.g. index.html, javascript.html

The icons in the web application are hand-sign, speech , upload file, send text

