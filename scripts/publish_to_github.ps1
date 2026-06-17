param(
  [string]$RemoteUrl = "https://github.com/lelala271/mobile-manipulation-nav-grasp.git"
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

git remote remove origin 2>$null
git remote add origin $RemoteUrl
git branch -M main
git push -u origin main
