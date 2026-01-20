from dataclasses import dataclass

@dataclass
class Event:
    source: str
    type: str
    timestamp: float
    payload: dict = None
    qr: str= None

@dataclass
class EventXLayer:
    workspace: dict
    event_id: dict
    mes_data: str
    good_items:int
    bad_items:int
    timestamp:int

EVENTS = {
    'QR_APPEND' : {"id": 10, "description_en": "Process started", "description_it": "Lavorazione iniziata"},
    'QR_REMOVED' : {"id": 11, "description_en": "Process suspended", "description_it": "Lavorazione sospesa"},
    'BUTTON_PRESSED' : {"id": 12, "description_en": "Process ended", "description_it": "Lavorazione conclusa"},
    'ENTER_DETECT' : {"id": 20, "description_en": "Item started", "description_it": "Articolo iniziato"},
    'EXIT_DETECT' : {"id": 21, "description_en": "Item finished", "description_it": "Articolo finito"}
}

WORKSPACES = {
    'PIPE_CUT' : {"id": 10, "description_en": "Pipe cutting", "description_it": "Taglio tubo"},
    'SHEET_CUT' : {"id": 11 , "description_en": "Sheet metal cutting", "description_it": "Taglio Lamiera"},
    'OVEN_1' : {"id": 20 , "description_en": "Oven and press", "description_it": "Forno e pressa"},
    'OVEN_2' : {"id": 21 , "description_en": "Oven and press", "description_it": "Forno e pressa"},
    'WELD_1' : {"id": 30 , "description_en": "Welding and grinding", "description_it": "Saldatura e molatura"},
    'WELD_2' : {"id": 31 , "description_en": "Welding and grinding", "description_it": "Saldatura e molatura"},
    'MILL_1' : {"id": 40 , "description_en": "CNC milling", "description_it": "Fresatura CNC"}
}

#(good_items, bad_items)
ITEMS = {
    'QR_APPEND' : (0,0),
    'QR_REMOVED' : (0,0),
    'BUTTON_PRESSED' : (0,0),
    'ENTER_DETECT' : (1,0),
    'EXIT_DETECT' : (1,0)
}
