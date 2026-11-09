#!/bin/bash
# opens the GUI in headless Brave/Chrome against a local server and prints the self-test result.   bash tests/run_gui_selftest.sh
cd "$(dirname "$0")/.."; PORT=8766
python3 -m http.server $PORT --bind 127.0.0.1 >/dev/null 2>&1 & SRV=$!; sleep 1
BR=$(command -v brave-browser || command -v chromium || command -v google-chrome)
timeout 120 "$BR" --headless=new --disable-gpu --no-sandbox --virtual-time-budget=25000 --dump-dom \
  "http://127.0.0.1:$PORT/tag_graph_maker.html?selftest=1&map=testmap/warehouse_slam.yaml&auto=1&frame=13.55,-10,0" 2>/dev/null \
  | python3 -c "import sys,re,html; d=sys.stdin.read(); m=re.search(r'<pre id=\"selftest\">(.*?)</pre>',d,re.S); print(html.unescape(m.group(1)) if m else 'NO SELFTEST OUTPUT (page error?)')"
kill $SRV 2>/dev/null
