import xml.etree.ElementTree as ET
import pytest
from raicom_control.field_tool_recovery import materialize_recovery_tree, recovery_tree_xml

TREE = '''<root main_tree_to_execute="MainTree"><BehaviorTree ID="MainTree">
<RecoveryNode name="NavigateRecovery" number_of_retries="6">
<RecoveryNode name="FollowPath" number_of_retries="1"><FollowPath/><ClearEntireCostmap service_name="local/clear"/></RecoveryNode>
<RoundRobin><ClearEntireCostmap service_name="global/clear"/><Spin spin_dist="1.57"/><Wait wait_duration="5"/></RoundRobin>
</RecoveryNode></BehaviorTree></root>'''


@pytest.mark.parametrize('limit',[0,2,10,23])
def test_only_outer_budget_changes_and_original_is_preserved(tmp_path,limit):
    template=tmp_path/'original.xml'; template.write_text(TREE)
    path=materialize_recovery_tree(limit,template=template,cache_dir=tmp_path/'cache')
    assert template.read_text()==TREE
    assert path==materialize_recovery_tree(limit,template=template,cache_dir=tmp_path/'cache')
    root=ET.parse(path).getroot()
    outer=root.find('./BehaviorTree/RecoveryNode')
    assert outer.get('number_of_retries')==str(limit+1)
    outer.set('number_of_retries','6')
    assert ET.tostring(root)==ET.tostring(ET.fromstring(TREE))


def test_unknown_tree_fails_instead_of_silently_using_six_retries():
    with pytest.raises(ValueError): recovery_tree_xml('<root/>',10)
