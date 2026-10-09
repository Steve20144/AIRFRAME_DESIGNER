"""Audited operator recovery, separate from the fenced worker completion protocol."""
import argparse, hashlib, json, os
from pathlib import Path
from .core import Store
from .worker import Mailbox

def regular(root, relative):
    rel = Path(relative)
    if rel.is_absolute() or ".." in rel.parts:
        raise ValueError("Artifact path escapes workspace")
    path = root / rel
    if any(p.is_symlink() for p in (path, *path.parents)) or not path.is_file():
        raise ValueError("Artifacts must be regular files without symlinks")
    if not path.resolve().is_relative_to(root.resolve()):
        raise ValueError("Artifact path escapes workspace")
    return path

def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()

def reconcile(store, mailbox, task_id, invocation, worker, manifest_rel, manifest_hash,
              raw, confirmation, process_check, evidence):
    if not confirmation.strip() or not process_check.strip():
        raise ValueError("Explicit quiescence confirmation and process audit required")
    with mailbox.lock():
        offer = mailbox.read("offer.json")
        if not offer or any(offer.get(k) != v for k,v in dict(task_id=task_id, invocation=invocation, worker_id=worker).items()):
            raise ValueError("Exact offered invocation required")
        if not offer.get("claimed"):
            raise ValueError("Only previously claimed work can be salvaged")
        with store.tx() as c:
            task = store.get(c, "tasks", task_id)
            goal = store.get(c, "goals", task["goal_id"])
        if goal["id"] != offer["goal_id"]:
            raise ValueError("Goal mismatch")
        root = Path(goal["workspace"])
        path = regular(root, manifest_rel)
        if digest(path) != manifest_hash:
            raise ValueError("Manifest changed")
        manifest = json.loads(path.read_text())
        if manifest["goal_id"] != goal["id"] or manifest["worker_id"] != worker:
            raise ValueError("Artifact identity mismatch")
        hashes = {str(path.relative_to(root)):manifest_hash}
        for row in manifest["outputs"]:
            artifact = regular(path.parent, row["path"])
            if digest(artifact) != row["sha256"]:
                raise ValueError("Artifact changed: " + row["path"])
            hashes[str(artifact.relative_to(root))] = row["sha256"]
        audit = dict(worker_id=worker, confirmation=confirmation, process_check=process_check,
                     artifact_hashes=hashes, receipt=invocation)
        receipt = store.reconcile_external_result(task_id, invocation, raw, offer["allowed_agents"], audit, evidence)
        # SQLite commit precedes mailbox release. Crash here keeps the old lease closed to new
        # work; rerunning the identical audit is idempotent and completes the release.
        offer.update(closed=True, reconciliation=receipt)
        mailbox.write("offer.json", offer)
        presence = mailbox.read("presence.json")
        if presence and presence.get("worker_id") == worker:
            presence["connected"] = False
            mailbox.write("presence.json", presence)
        return receipt

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--state",type=Path,required=True)
    p.add_argument("--audit",type=Path,required=True,help="Reviewed operator JSON; never accepted from worker output")
    a=p.parse_args(); os.umask(0o077)
    audit=json.loads(a.audit.read_text())
    store=Store(a.state / "factory.sqlite3")
    print(json.dumps(reconcile(store, Mailbox(a.state,audit.pop("agent")), **audit)))

if __name__ == "__main__": main()
