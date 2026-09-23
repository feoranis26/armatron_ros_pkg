"""Humble RewrittenYaml compatibility for scalar leaves inside arrays."""
from nav2_common.launch import RewrittenYaml as Nav2RewrittenYaml


class RewrittenYaml(Nav2RewrittenYaml):
    def updateYamlPathVals(self, yaml, yaml_key_list, rewrite_val):
        # Humble's implementation converts intermediate list indices, but uses
        # a string for the final index. Keep existing-path semantics and handle
        # the leaf exactly like every intermediate element.
        target = yaml
        for key in yaml_key_list[:-1]:
            target = target[int(key)] if isinstance(target, list) else target[key]
        key = yaml_key_list[-1]
        target[int(key) if isinstance(target, list) else key] = rewrite_val
        return yaml
