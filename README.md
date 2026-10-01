# 手続型惑星地形生成器

Unreal Engine 5.8 と C++ による手続型惑星地形生成器の開発版です．Seed と惑星半径を設定し，球面上の地質・河川・気候・植生・昼夜・季節を組み合わせます．

## 起動

Windows に Unreal Engine 5.8 と Visual Studio の C++ 開発環境を導入し，`PlanetGenerator.uproject` を開いて下さい．
省メモリ設定でエディタを起動する場合は `Scripts/UE/Start-Editor.ps1` を使用します．
探索レベルは `/Game/Planet/Maps/Lvl_Planet` です．

## リリース

GitHub Actions は `v*` タグの push のみで起動します．
Core 契約試験の後，Unreal Engine 5.8 を導入したラベル `ue-5-8` の Windows セルフホストランナーで Win64 版をパッケージします．
パッケージに成功した場合にだけ GitHub Release へ添付します．
