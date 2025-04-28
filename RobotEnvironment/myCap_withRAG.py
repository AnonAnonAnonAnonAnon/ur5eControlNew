#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import sys
import re
import argparse
import base64
import runpy
from openai import OpenAI
import shutil

# --------------------------------------------
# VLM 调用封装
# --------------------------------------------
def encode_image(image_path: str) -> str:
    """Base64 编码图片，返回字符串。"""
    with open(image_path, "rb") as f:
        return base64.b64encode(f.read()).decode("utf-8")

class VLMClient:
    def __init__(self, api_key: str, base_url: str, model: str, temperature: float, max_tokens: int):
        self.client = OpenAI(base_url=base_url, api_key=api_key)
        self.model = model
        self.temperature = temperature
        self.max_tokens = max_tokens

    def query(self, instruction: str, prompt_template_path: str, image_path: str) -> str:
        # 1) 读取并格式化 prompt
        with open(prompt_template_path, "r", encoding="utf-8") as f:
            template = f.read()
        prompt_text = template.format(instruction=instruction)

        # 2) 编码图片
        img_b64 = encode_image(image_path)

        # 3) 构造 messages
        messages = [
            {
                "role": "user",
                "content": [
                    { "type": "text",      "text": prompt_text },
                    { "type": "image_url", "image_url": { "url": f"data:image/png;base64,{img_b64}" } }
                ]
            }
        ]

        # 4) 调用 VLM API（stream 模式）
        stream = self.client.chat.completions.create(
            model=self.model,
            messages=messages,
            temperature=self.temperature,
            max_tokens=self.max_tokens,
            stream=True
        )

        # 5) 收流并拼接（跳过没有 choices 的 chunk）
        output = ""
        for chunk in stream:
            if not getattr(chunk, "choices", None):
                continue
            choice = chunk.choices[0]
            delta = getattr(choice, "delta", None)
            if not delta or delta.content is None:
                continue
            output += delta.content
        return output  # 如果完全没有流到内容，也会返回空字符串

# --------------------------------------------
# 从 VLM 原始输出中提取纯代码
# --------------------------------------------
def extract_code_block(vlm_output) -> str:
    """
    支持提取 ```python ...``` 或 ```...``` 之间的内容；
    如未找到，直接把整个输出当作代码返回。
    即使传入 None，也会安全返回空字符串。
    """
    if not isinstance(vlm_output, str):
        return ""
    pattern = r"```(?:python)?\s*([\s\S]*?)```"
    m = re.search(pattern, vlm_output, flags=re.IGNORECASE)
    if m:
        return m.group(1)
    return vlm_output

# --------------------------------------------
# 主流程
# --------------------------------------------
def main():
    p = argparse.ArgumentParser(description="查询 VLM，提取代码并执行")
    p.add_argument("--instruction", required=False,
                   default="place bottle on pen",
                   help="文本形式的指令")
    p.add_argument("--prompt",      required=False,
                   default="/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/cap_prompt_template.txt",
                   help="prompt 模板文件路径")
    p.add_argument("--image",       required=False,
                   default="/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/query_image.png",
                   help="图片文件路径")
    args = p.parse_args()

    # 确保当前目录能 import 本地模块
    sys.path.insert(0, os.getcwd())

    # 准备历史记录目录
    history_dir = "/home/liwenbo/project/yt/ur5eControlNew/RobotEnvironment/exec_history"
    os.makedirs(history_dir, exist_ok=True)

    # 关键词分词
    keywords = re.findall(r"\w+", args.instruction.lower())
    print(f"[DEBUG] keywords: {keywords}")

    # 计算每个历史脚本的匹配得分
    scores = {}
    for fname in os.listdir(history_dir):
        if not fname.endswith(".py"):
            continue
        name = os.path.splitext(fname)[0].lower()
        score = sum(name.count(kw) for kw in keywords)
        if score > 0:
            scores[fname] = score
    # 取前 N 个最匹配脚本
    N = 3
    selected = sorted(scores, key=lambda f: scores[f], reverse=True)[:N]
    print(f"[DEBUG] scores: {scores}")
    print(f"[DEBUG] selected: {selected}")
    # 读取选中脚本内容
    history_texts = []
    for fname in selected:
        path = os.path.join(history_dir, fname)
        with open(path, "r", encoding="utf-8") as f:
            history_texts.append(f.read())
    history_txt = "\n\n".join(history_texts)
    print(f"[DEBUG] history_txt: {history_txt}")

    # 读取原始 prompt 模板
    with open(args.prompt, "r", encoding="utf-8") as f:
        prompt_template = f.read()

    # 合并历史与原模板，生成新的 prompt 文件
    merged_prompt = (
        "### Context from history scripts:\n" +
        history_txt +
        "\n\n### Original prompt template:\n" +
        prompt_template
    )
    print(f"[DEBUG] merged_prompt: {merged_prompt}")
    merged_prompt_file = "merged_prompt.txt"
    with open(merged_prompt_file, "w", encoding="utf-8") as f:
        f.write(merged_prompt)

    # 硬编码 API Key
    api_key = "sk-7h6cOfz7m4hIRwIp3MAliy4WKTfB1oHtRa0kv29fsJu3bOd7"
    vlm = VLMClient(
        api_key=api_key,
        base_url="https://api.chatanywhere.tech",
        model="chatgpt-4o-latest",
        temperature=0.0,
        max_tokens=2048
    )

    print("[INFO] 开始向 VLM 提交任务…")
    print(f"[DEBUG] merged_prompt_file: {merged_prompt_file}")
    raw_output = vlm.query(args.instruction, merged_prompt_file, args.image)
    print("[INFO] VLM 原始返回：")
    print(raw_output)

    code = extract_code_block(raw_output)
    if not code.strip():
        print("[WARN] 未从 VLM 返回中提取到任何代码，退出。")
        sys.exit(1)

    print("[INFO] 已提取代码段：")
    print(code)

    # 写入独立执行文件
    out_file = "vlm_generated.py"
    with open(out_file, "w", encoding="utf-8") as f:
        f.write(code)
    print(f"[INFO] 代码已保存至 {out_file}")

    # 备份到历史目录
    task_name = re.sub(r"\W+", "_", args.instruction.strip().lower()).strip("_")
    history_file = os.path.join(history_dir, f"{task_name}.py")
    shutil.copy(out_file, history_file)
    print(f"[INFO] 已将脚本备份至 {history_file}")

    ############################# 执行 ################################
    print(f"[INFO] 正在执行 {out_file} …")
    # runpy.run_path(out_file, run_name="__main__")
    print("[INFO] 执行流程完成(此处未实际执行脚本)。")

if __name__ == "__main__":
    main()
