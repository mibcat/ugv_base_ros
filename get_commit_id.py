import subprocess

Import("env")


def get_git_commit_hash():
    ret = subprocess.run(["git", "rev-parse", "--short=6", "HEAD"], stdout=subprocess.PIPE, text=True)
    hash_string = ret.stdout.strip()
    build_flag = '-D GIT_COMMIT_HASH=\\"' + hash_string + '\\"'
    print("git commit hash: " + hash_string)
    return build_flag


env.Append(BUILD_FLAGS=[get_git_commit_hash()])
