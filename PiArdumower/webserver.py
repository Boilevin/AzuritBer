from flask import Flask
app= Flask(__name__)

@app.route("/")
def hello():
    return "coucou"
if __name__ == "__main__":
    app.run(host="0.0.0.0",port=80,debug=True)

    
