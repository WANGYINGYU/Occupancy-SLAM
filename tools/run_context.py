"""Durable stage status for complete, failed and interrupted pipeline runs."""
import json,time
class RunContext:
    def __init__(self,output): self.path=output/'run_status.json';self.started=time.monotonic()
    def update(self,**values):
        state=json.loads(self.path.read_text()) if self.path.exists() else {}
        state.update(values);self.path.write_text(json.dumps(state,indent=2))
    def __enter__(self):self.update(status='running',stage='input');return self
    def __exit__(self,kind,error,trace):
        self.update(status='completed' if kind is None else 'interrupted' if issubclass(kind,KeyboardInterrupt) else 'failed',error=str(error) if error else None,elapsed_s=time.monotonic()-self.started)
