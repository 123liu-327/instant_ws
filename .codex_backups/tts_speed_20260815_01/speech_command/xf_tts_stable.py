import websocket
import datetime
import hashlib
import base64
import hmac
import json
import os
import sys
from urllib.parse import urlencode

class Ws_Param(object):
    def __init__(self, APPID, APIKey, APISecret, Text):
        self.APPID, self.APIKey, self.APISecret, self.Text = APPID, APIKey, APISecret, Text
        self.CommonArgs = {"app_id": self.APPID}
        self.BusinessArgs = {"aue": "raw", "auf": "audio/L16;rate=16000", "vcn": "xiaoyan", "tte": "utf8"}
        self.Data = {"status": 2, "text": str(base64.b64encode(self.Text.encode('utf-8')), "utf-8")}

    def create_url(self):
        url = 'wss://tts-api.xfyun.cn/v2/tts'
        now = datetime.datetime.utcnow().strftime('%a, %d %b %Y %H:%M:%S GMT')
        signature_origin = "host: tts-api.xfyun.cn\n"
        signature_origin += "date: " + now + "\n"
        signature_origin += "GET /v2/tts HTTP/1.1"
        signature_sha = hmac.new(self.APISecret.encode('utf-8'), signature_origin.encode('utf-8'), digestmod=hashlib.sha256).digest()
        signature_sha = base64.b64encode(signature_sha).decode(encoding='utf-8')
        # 严格无空格校验拼接
        authorization_origin = f'api_key="{self.APIKey}",algorithm="hmac-sha256",headers="host date request-line",signature="{signature_sha}"'
        authorization = base64.b64encode(authorization_origin.encode('utf-8')).decode(encoding='utf-8').replace('\n', '')
        v = {"authorization": authorization, "date": now, "host": "tts-api.xfyun.cn"}
        return url + '?' + urlencode(v)

def on_message(ws, message):
    try:
        res = json.loads(message)
        if res["code"] != 0: ws.close(); return
        audio = base64.b64decode(res["data"]["audio"])
        # 严格保存到 C++ 指定的包内部路径
        with open('/home/ucar/instant_ws/src/speech_command/tmp/tts_result.pcm', 'ab') as f: f.write(audio)
        if res["data"]["status"] == 2: ws.close()
    except: pass

def on_error(ws, error): pass
def on_close(ws,a,b): pass
def on_open(ws):
    req = {"common": wsParam.CommonArgs, "business": wsParam.BusinessArgs, "data": wsParam.Data}
    ws.send(json.dumps(req))

if __name__ == "__main__":
    target_dir = os.path.dirname('/home/ucar/instant_ws/src/speech_command/tmp/tts_result.pcm')
    if not os.path.exists(target_dir): os.makedirs(target_dir)
    wsParam = Ws_Param(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4])
    ws = websocket.WebSocketApp(wsParam.create_url(), on_message=on_message, on_error=on_error, on_close=on_close)
    ws.on_open = on_open
    ws.run_forever()
