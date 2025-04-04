FROM python:3.10-slim

WORKDIR /app

COPY backend/requirements.txt .

RUN pip install -r requirements.txt

COPY backend/ .

CMD ["python", "fastlio_backend.py"]