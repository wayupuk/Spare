from fastapi import FastAPI, Request, Form
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates
from typing import List, Optional
from pydantic import BaseModel
from typing import Optional
from importlib.resources import files
import logging
from logging.handlers import RotatingFileHandler
import os
from pathlib import Path
from asset.utils import S2S,RapidChangeWindowClassifier,CNNTimeSeriesClassifier
# from f5_tts import api as fapi
print(os.curdir)
import torch
import json
import torch.nn as nn
import torch.nn.functional as f
"-----------------------------------------"
DELTA_ACC_THRESHOLD = 0
DELTA_GYRO_THRESHOLD = 0
DELTA_FLEX_THRESHOLD = 0
WINDOW_SIZE = 2   # ใช้ย้อนหลัง 5 segment
ini_state = False
classifier = RapidChangeWindowClassifier(DELTA_ACC_THRESHOLD,DELTA_GYRO_THRESHOLD,DELTA_FLEX_THRESHOLD,WINDOW_SIZE)
model = CNNTimeSeriesClassifier((50,28),51)
model_name = r"F:\Hybridmodel-project\Sign_Language_Detection\fastapi_docker_webapp\app\asset\model\KhanomTanLLM-1B"
seq = S2S(model_name)
predictions = []
data = []
ft = []
sta = []
state = False
a = []
thes = 5
model_path = r"F:\Hybridmodel-project\Sign_Language_Detection\fastapi_docker_webapp\app\asset\model\model_89.pth"
if torch.cuda.is_available():
    weight = torch.load(model_path,weights_only=True)
    model.load_state_dict(weight)
    device = "cuda"
else:
    weight = torch.load(model_path,weights_only=False,map_location=torch.device('cpu'))
    model.load_state_dict(weight)
    device = "cpu"
    

model.to(device)
model.double()
model.eval()

with open("./asset/rollback.json","r",encoding='utf-8') as f:
        content = json.load(f)

# f5_tts = api.F5TTS()

print("Ready to use")
"------------------------------------------"
APP_DIR = Path(__file__).resolve().parent
LOG_DIR = APP_DIR / "logs"
LOG_DIR.mkdir(parents=True, exist_ok=True)
LOG_FILE = LOG_DIR / "app.log"

logger = logging.getLogger("app_logger")
logger.setLevel(logging.INFO)
handler = RotatingFileHandler(LOG_FILE, maxBytes=2_000_000, backupCount=3, encoding="utf-8")
formatter = logging.Formatter('%(asctime)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

app = FastAPI()

if (APP_DIR / "static").exists():
    app.mount("/static", StaticFiles(directory=str(APP_DIR / "static")), name="static")

templates = Jinja2Templates(directory=str(APP_DIR / "templates"))

from fastapi.middleware.cors import CORSMiddleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_methods=["*"],
    allow_headers=["*"],
)

class PredictRequest(BaseModel):
    feature: List[List[float]]
    metadata: Optional[dict] = None
    
class Text_voice(BaseModel):
    texts: List[str]
    metadata: Optional[dict] = None

@app.get("/", response_class=HTMLResponse)
async def index(request: Request):
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/logs", response_class=HTMLResponse)
async def view_logs(request: Request, lines: int = 200):
    n = max(1, min(10000, lines))
    if not LOG_FILE.exists():
        logs = "(log file not found yet)"
    else:
        with open(LOG_FILE, "r", encoding="utf-8", errors="ignore") as f:
            logs = "".join(f.readlines()[-n:])
    return templates.TemplateResponse("logs.html", {"request": request, "logs": logs})

@app.post("/predict")
async def predict_api(payload: PredictRequest):
    
    global predictions
    global data 
    global ft
    global sta
    global state 
    global a 
    global thes 
    
    
    logger.info(f"/predict called (api) with text={payload.feature}")
    # result = {"received": payload.feature, "length": len(payload.feature)}
    # logger.info(f"/predict result: {result}")
    rows = payload.feature
    logger.info(f"/aloha result: ",ft,predictions,data,sta,state)
    ft,predictions,data,sta,state = classifier.pred(rows,predictions,data,ft,state,sta)
    # logger.info(f"/aloha result-end: ",ft,predictions,data,sta,state)
    
    if state==True and len(ft)>=thes and len(predictions) > 10:
        
        sequence = classifier.padding(data,thes)
        # print(sequence)
        batch_x = torch.from_numpy(sequence)
        batch_x = batch_x[:,1:].unsqueeze(0)
        
        outputs = model(batch_x.double().to(device))
        text = content[str(torch.argmax(outputs).item())]
        result = {"received":text}
    else:
        result = {"received":len(data)}
    
    return JSONResponse(result)

@app.post("/predict-form")
async def predict_form(request: Request, text: str = Form(...)):
    logger.info(f"/predict called (form) with text={text}")
    result = {"received": text, "length": len(text)}
    logger.info(f"/predict-form result: {result}")
    return JSONResponse(result)


@app.post("/tts")
async def predict_form(request: Text_voice):
    
    global f5_tts
    
    result = {"texts":request.texts}
    output = seq.predict(request.texts)
    result["outputs"] = output
    
    # wav, sr, spect = f5_tts.infer(
    #     ref_file=str(files("f5_tts").joinpath("infer/examples/basic/basic_ref_en.wav")),
    #     ref_text="some call me nature, others call me mother nature.",
    #     gen_text=request.texts,
    #     # file_wave=str(files("f5_tts").joinpath("../../tests/api_out.wav")),
    #     # file_spect=str(files("f5_tts").joinpath("../../tests/api_out.png")),
    #     # seed=-1,  # random seed = -1
    # )
    
    
    return JSONResponse(result)


@app.get("/health")
async def health():
    return {"status": "ok"}

# if __name__ == '__main__':
#     app.run(host='0.0.0.0', port=5000, debug=True)
