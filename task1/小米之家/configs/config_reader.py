import configparser
import os

class _A:
    def __init__(self, _a="configs.ini"):
        self._b = _a
        self._c = {
            'a_wh': 'A-1',
            'b_wh': 'B-1', 
            'arr': 'L'
        }
        
    def _d(self):
        _e = configparser.ConfigParser()
        try:
            if os.path.exists(self._b):
                _e.read(self._b, encoding='utf-8')
                if 'CONTROL' in _e:
                    _f = {}
                    _f['a_wh'] = _e.get('CONTROL', 'a_warehouse', fallback=self._c['a_wh']).strip()
                    _f['b_wh'] = _e.get('CONTROL', 'b_warehouse', fallback=self._c['b_wh']).strip()
                    _f['arr'] = _e.get('CONTROL', 'arrow', fallback=self._c['arr']).strip()
                    return _f
        except Exception:
            pass
        
        return self._c