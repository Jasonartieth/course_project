# course_project — 项目说明

- 创建日期：2026-10-22
- GitHub：Jasonartieth/course_project（私有）
- 主题：待定

## 目录约定
- 代码 → `scripts/`；数据 → `data/`；产物 → `results/<时间戳或序号>/`；日志 → `logs/`；文档 → `docs/`
- 不再使用但不删的大文件 → `_backup/<任务名>_<日期>/`（不进 git，仓库 archive 里留清单+路径指针）

## 进展
- 2026-10-22：建立项目骨架与私有仓

## 已知坑 / ★
- ★ git push 报 "Repository not found"：本机 SSH key 绑定的是 kefusong 账号，访问不到 Jasonartieth 的私有仓 → remote 必须用 HTTPS（`https://github.com/Jasonartieth/course_project.git`，走 gh 凭据）。`gh repo create --source=. --push` 默认用 SSH，会失败，需要先 `git remote set-url` 改成 HTTPS。

## 归档索引（任务 → 仓库/分支/tag/路径）
