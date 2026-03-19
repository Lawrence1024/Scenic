# Load the CoSim-only OSA into VEOS (run after Docker restart).
#
# Requires: veos container running; no other app holding the simulator (e.g. ControlDesk/XIL).
# The container uses port mapper at 111 (VEOS_COSIM_PORTMAPPER_PORT); from host use e.g. 11111:111.
# See README.md for compose ports (mapper 111, symmetric CoSim server range, optional 2017 for AURELION).
#
# Usage: .\load_cosim_osa.ps1

$ErrorActionPreference = "Stop"
$cosimDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$exampleDir = Join-Path $cosimDir "CoSIm_Client_Example_Cpp"
$jsonPath = Join-Path $exampleDir "cosim_server_config.json"
$schemaPath = Join-Path $exampleDir "DsVeosCoSim.schema.json"
$container = "veos"
$veosBin = "/opt/dspace/veos2023a/bin"
$osaPathInTmp = "/tmp/DsVeosCoSim.osa"   # where veos model import writes the OSA
$osaPathInVeos = "/home/dspace/VEOS/DsVeosCoSim.osa"   # optional copy for persistence

if (-not (Test-Path $jsonPath)) {
    Write-Error "cosim_server_config.json not found at $jsonPath"
}
if (-not (Test-Path $schemaPath)) {
    Write-Error "DsVeosCoSim.schema.json not found at $schemaPath"
}

Write-Host "Copying example CoSim JSON + schema into container..."
docker cp $jsonPath "${container}:/tmp/cosim_server_config.json"
docker cp $schemaPath "${container}:/tmp/DsVeosCoSim.schema.json"

Write-Host "Creating OSA from JSON..."
docker exec $container bash -c "cd /tmp && $veosBin/veos model import -n ./DsVeosCoSim.osa -p ./cosim_server_config.json"
# Optional: copy to VEOS dir for persistence (may need root if dir is root-owned)
docker exec $container bash -c "test -d /home/dspace/VEOS && cp /tmp/DsVeosCoSim.osa $osaPathInVeos 2>/dev/null || true"

Write-Host "Loading OSA into simulator..."
docker exec $container bash -c "$veosBin/veos-sim load $osaPathInTmp"

Write-Host "Starting simulation..."
docker exec $container bash -c "$veosBin/veos-sim start"

Write-Host "Done. CoSim server 'CoSimExample' should be available. Run clients with server name CoSimExample and port 11111."
