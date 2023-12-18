#!/bin/bash
## 直接创建分支并推送远程
# git checkout -b test001
# git push origin test001
## 删除本地分支
# git branch -d test001
## 删除某个远程分支
# git push origin --delete branch_name
## 更新完某个bash文件后, 需要执行下面的命令使其生效
# chmod +x branch_add.sh
##然后运行
# ./branch_add.sh
# 通过读取csv创建分支
###################################################
# CSV 文件路径
# csv_file="./branchName.csv"

# # 读取 CSV 文件中的分支列表
# branches=($(csvtool -t COMMA col 1 $csv_file))
###################################################
# 通过写名字创建
branches=("motor-test" "simulink" "arrange-T")
# ###################################################
# 远程仓库的名称
remote="origin"

# 切换到主分支（或其他基础分支）
base_branch="main"

# 循环遍历推送分支到远程仓库
for branch in "${branches[@]}"; do
    # 推送分支到远程仓库
    git push "$remote" "$base_branch:$branch"
done