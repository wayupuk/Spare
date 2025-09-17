# FastAPI Docker Web App

Build and run with Docker Compose (from project root):

```bash
docker compose up --build
```

Application will be available at `http://localhost:8000/`.

- `GET /` : interactive page to send text to `/predict`
- `POST /predict` : JSON API, body `{ "text": "..." }`
- `POST /predict-form` : Form POST (used by the interactive page)
- `GET /logs` : web log viewer (last 200 lines by default) 

Logs are written to `app/logs/app.log` and rotated.



# FastAPI roun with Uvicorn

```bash
pip install uvicorn
```

```bash
uvicorn main:app --host 0.0.0.0 --port 80 --reload
```
to run with uvicorn


# Run global with ngrok

Install Ngrok via:

```bash
ngrok config add-authtoken <authen-token>

ngrok http http://localhost:$PORT ### ngrok http http://localhost:$PORT
```
