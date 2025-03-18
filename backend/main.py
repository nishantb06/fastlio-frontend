from mangum import Mangum
from backend.fastlio_backend import app

# Create handler for AWS Lambda
handler = Mangum(app) 