from mangum import Mangum
from main import app

# Create handler for AWS Lambda
handler = Mangum(app) 