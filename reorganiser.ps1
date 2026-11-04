# Reorganisation du depot SLAM
# 1. Placer ce fichier dans le dossier SLAM (celui qui contient .git)
# 2. Dans ce dossier : clic droit > "Ouvrir dans le terminal", puis :
#      powershell -ExecutionPolicy Bypass -File .\reorganiser.ps1
# Ne modifie aucun contenu : uniquement des deplacements / renommages.

$ErrorActionPreference = 'Stop'
Set-Location -LiteralPath $PSScriptRoot

if (-not (Test-Path -LiteralPath '.git')) {
    Write-Host "Lance ce script depuis le dossier SLAM (la ou il y a .git)" -ForegroundColor Red
    exit 1
}

# Deplace les fichiers correspondant aux motifs vers $dest, sans rien ecraser
function Deplacer([string]$dest, [string[]]$motifs) {
    $fichiers = @(foreach ($m in $motifs) { Get-ChildItem -File -Filter $m }) |
        Sort-Object -Property FullName -Unique
    if (-not $fichiers) { return }
    New-Item -ItemType Directory -Force -Path $dest | Out-Null
    foreach ($f in $fichiers) {
        $cible = Join-Path $dest $f.Name
        if (Test-Path -LiteralPath $cible) {
            Write-Warning "$($f.FullName) existe deja dans $dest -> non deplace"
        } else {
            Move-Item -LiteralPath $f.FullName -Destination $cible
        }
    }
}

function Renommer([string]$ancien, [string]$nouveau) {
    if (Test-Path -LiteralPath $ancien) { Rename-Item -LiteralPath $ancien -NewName $nouveau }
}

# Range les sketchs Arduino : firmware/<nom>/<nom>.ino
function RangerFirmware {
    foreach ($f in @(Get-ChildItem -File -Filter '*.ino')) {
        $nom = $f.BaseName
        $dossier = Join-Path 'firmware' $nom
        if ((Test-Path -LiteralPath $nom) -or (Test-Path -LiteralPath $dossier)) {
            Write-Warning "$($f.FullName) : un dossier '$nom' existe deja -> comparer puis supprimer le doublon"
        } else {
            New-Item -ItemType Directory -Force -Path $dossier | Out-Null
            Move-Item -LiteralPath $f.FullName -Destination (Join-Path $dossier $f.Name)
        }
    }
    foreach ($d in 'ESP32_LD06_WiFi', 'ESP32_LD06_Capture') {
        if (Test-Path -LiteralPath $d -PathType Container) {
            New-Item -ItemType Directory -Force -Path 'firmware' | Out-Null
            Move-Item -LiteralPath $d -Destination (Join-Path 'firmware' $d)
        }
    }
}

Write-Host "1. Renommage des etapes"
Renommer 'Debug moteur'                    '01_debug_moteur'
Renommer 'Cartographie'                    '02_cartographie'
Renommer 'Localisation'                    '03_localisation_exploration'
Renommer 'cartographie avec les obstacles' '04_cartographie_obstacles'
Renommer 'Solution final'                  '05_resultat_final'

Write-Host "2. Rapport"
Deplacer 'rapport' @('Rapport_*.odt')

Write-Host "3. Etape 02 - cartographie"
Push-Location '02_cartographie'
Renommer 'TP_LD06_cartographie (6)' 'TP_LD06_cartographie'
RangerFirmware
Deplacer 'docs'      @('DEMARCHE_*.md')
Deplacer 'resultats' @('resultats_*', '*.png', 'carte_vectorielle*')
Deplacer 'data'      @('*.json', '*.csv')
Pop-Location

Write-Host "4. Etape 03 - localisation / exploration"
Push-Location '03_localisation_exploration'
Get-ChildItem -File -Filter '*.crdownload' | Remove-Item
RangerFirmware
Deplacer 'docs'      @('DEMARCHE_*.md')
Deplacer 'resultats' @('resultats_*', '*.png')
Deplacer 'data'      @('*.json')
Pop-Location

Write-Host "5. Etape 04 - cartographie avec obstacles"
Push-Location '04_cartographie_obstacles'
Deplacer 'data' @('*.csv')
Pop-Location

Write-Host "6. Etape 05 - resultat final"
Push-Location '05_resultat_final'
Renommer 'session_lidar (1).json' 'session_lidar.json'
Pop-Location

Write-Host "7. .gitignore"
if (-not (Test-Path -LiteralPath '.gitignore')) {
    @(
        '# Telechargements incomplets', '*.crdownload',
        '# Fichiers systeme', 'Thumbs.db', 'desktop.ini', '.DS_Store',
        '# Python', '__pycache__/', '*.pyc',
        '# Editeurs', '.vscode/'
    ) | Set-Content -LiteralPath '.gitignore' -Encoding ascii
}

Write-Host ""
Write-Host "Termine. A verifier a la main avant le commit :" -ForegroundColor Green
Write-Host "  - 02_cartographie : lidar_maptools.js vs 'lidar_maptools (1).js' -> garder la bonne"
Write-Host "  - 02_cartographie : ESP32_LD06_WiFi.ino isole vs firmware\ESP32_LD06_WiFi\ -> supprimer le doublon"
Write-Host "  - ouvrir lidar_viewer.html (etapes 02 et 03) et lancer build.py pour verifier"
Write-Host "Puis : git add -A ; git status"
