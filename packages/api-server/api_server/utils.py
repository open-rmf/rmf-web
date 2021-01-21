def urlpathjoin(*parts) -> str:
    parts = [s.strip('/') for s in parts]
    parts = [s for s in parts if len(s)]
    return '/' + '/'.join(parts)
