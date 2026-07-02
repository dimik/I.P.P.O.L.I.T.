import os, qai_appbuilder
from qai_appbuilder import QNNContext, QNNConfig, Runtime, LogLevel, ProfilingLevel
libs = os.path.join(os.path.dirname(qai_appbuilder.__file__), "libs")
os.environ.setdefault("ADSP_LIBRARY_PATH", libs)   # so the v68 skel loads onto the cDSP
BIN = "/home/radxa/llama-1b/models/weight_sharing_model_1_of_1.serialized.bin"
print("qai_appbuilder libs:", libs)
print("=== QNNConfig.Config(Runtime.HTP) ===")
QNNConfig.Config(Runtime.HTP, LogLevel.WARN, ProfilingLevel.BASIC)
print("=== loading v68 context binary via QNN-direct (no Genie) ===")
m = QNNContext("llama", BIN)
print(">>> LOADED ON NPU (HTP) OK — QNN-direct path works")
try:
    print("graph name:", m.getGraphName())
    print("input names:", m.getInputName())
    print("input shapes:", m.getInputShapes())
    print("input dtypes:", m.getInputDataType())
    print("output names:", m.getOutputName())
    print("output shapes:", m.getOutputShapes())
except Exception as e:
    print("IO introspection:", type(e).__name__, e)
