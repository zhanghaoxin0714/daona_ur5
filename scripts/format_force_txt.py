import argparse
import os
import re
from typing import List


FLOAT_PATTERN = re.compile(r"[-+]?(?:\d*\.\d+|\d+)(?:e[-+]?\d+)?", re.IGNORECASE)


def extract_numbers(text: str) -> List[str]:
    """
    Extract all float-like number strings from the given text.
    We keep original string representations (no float conversion) to avoid
    introducing rounding differences.
    """
    return FLOAT_PATTERN.findall(text)


def group_every_six(values: List[str]) -> List[List[str]]:
    """
    Group a flat list of string numbers into chunks of six.
    Any trailing remainder < 6 is ignored (and returned separately for reporting).
    """
    grouped: List[List[str]] = []
    for i in range(0, len(values) - (len(values) % 6), 6):
        grouped.append(values[i : i + 6])
    return grouped


def main() -> None:
    parser = argparse.ArgumentParser(
        description="格式化六维力数据：去除括号等符号，每行6个数，用逗号分隔。"
    )
    parser.add_argument(
        "--input",
        "-i",
        required=False,
        default=r"C:\Users\张皓鑫\Desktop\ORU数据\六维力数据\oru_force.txt",
        help="输入txt文件路径（包含原始六维力数据）。",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=r"C:\Users\张皓鑫\Desktop\ORU数据\六维力数据\oru_force_formatted.txt",
        help="输出txt文件路径（每行六个数据，逗号分隔）。默认与输入同目录，文件名加 _formatted 后缀。",
    )
    args = parser.parse_args()

    input_path = os.path.abspath(args.input)
    if args.output:
        output_path = os.path.abspath(args.output)
    else:
        root, ext = os.path.splitext(input_path)
        output_path = f"{root}_formatted{ext or '.txt'}"

    if not os.path.exists(input_path):
        raise FileNotFoundError(f"输入文件不存在: {input_path}")

    with open(input_path, "r", encoding="utf-8", errors="ignore") as f:
        content = f.read()

    numbers = extract_numbers(content)
    grouped = group_every_six(numbers)

    # 写入结果
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, "w", encoding="utf-8") as out:
        for row in grouped:
            out.write(",".join(row) + "\n")

    # 控制台提示
    remainder = len(numbers) % 6
    print(f"已写入: {len(grouped)} 行 -> {output_path}")
    if remainder != 0:
        print(
            f"注意：共有 {len(numbers)} 个数字，不能被6整除，末尾剩余 {remainder} 个已忽略。"
        )


if __name__ == "__main__":
    main()

