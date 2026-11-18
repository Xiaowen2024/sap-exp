"""Memory capacities only, for Genesis 1.4.6's native SAP TET handlers.

Same structs, detection and solve kernels as upstream. Lower initial capacities are
retried on explicit overflow; contact truncation is never accepted. No mesh or
numerical contact/solver parameter changes. Original allocation formulas are in
v1.4.6/genesis/engine/couplers/sap_coupler.py (Apache-2.0).
"""

import genesis as gs
import quadrants as qd
import genesis.engine.couplers.sap_coupler as sap

@qd.kernel
def safe_update_contact(self: qd.template(), i_step: qd.i32,
    links_info: sap.LinksInfo, faces_info: sap.FacesInfo, verts_info: sap.VertsInfo,
    free_verts_state: sap.VertsState, fixed_verts_state: sap.VertsState,
    geoms_state: sap.GeomsState, geoms_info: sap.GeomsInfo,
    dofs_state: sap.DofsState, links_state: sap.LinksState,
    contact_queries_state: sap.SAPContactQueriesState) -> tuple[bool,bool]:
    has_contact=False
    overflow=False
    for contact in qd.static(self.contact_handlers):
        local_overflow=contact.detection(i_step,links_info=links_info,verts_info=verts_info,faces_info=faces_info,free_verts_state=free_verts_state,fixed_verts_state=fixed_verts_state,geoms_state=geoms_state,geoms_info=geoms_info,contact_queries_state=contact_queries_state)
        overflow |= local_overflow
        has_contact |= contact.n_contact_pairs[None]>0
        if not local_overflow:
            contact.compute_jacobian(links_info=links_info,dofs_state=dofs_state,links_state=links_state)
    return has_contact,overflow

def configure(capacity=8192, query_factor=2, pair_factor=2):
 import genesis as gs
 import quadrants as qd
 import genesis.engine.couplers.sap_coupler as sap

 def initialize(self,simulator,eps=1e-10):
  sap.RigidContactHandler.__init__(self,simulator)
  pair=isinstance(self,sap.RigidRigidTetContactHandler)
  self.name='RigidRigidTetContactHandler' if pair else 'RigidFloorTetContactHandler'
  self.eps=eps
  if pair:
   self.contact_candidate_type=qd.types.struct(batch_idx=gs.qd_int,geom_idx0=gs.qd_int,geom_idx1=gs.qd_int,intersection_code0=gs.qd_int,normal=gs.qd_vec3,x=gs.qd_vec3,distance0=gs.qd_vec4)
   self.contact_pair_type=qd.types.struct(batch_idx=gs.qd_int,normal=gs.qd_vec3,tangent0=gs.qd_vec3,tangent1=gs.qd_vec3,link_idx0=gs.qd_int,link_idx1=gs.qd_int,contact_pos=gs.qd_vec3,sap_info=self.sap_contact_info_type)
  else:
   self.contact_candidate_type=qd.types.struct(batch_idx=gs.qd_int,geom_idx=gs.qd_int,intersection_code=gs.qd_int,distance=gs.qd_vec4)
   self.contact_pair_type=qd.types.struct(batch_idx=gs.qd_int,link_idx=gs.qd_int,contact_pos=gs.qd_vec3,sap_info=self.sap_contact_info_type)
  n=min(self.coupler.rigid_volume_elems.shape[0]*pair_factor,capacity)*self.sim._B
  self.n_contact_candidates=qd.field(gs.qd_int,shape=())
  self.max_contact_candidates=n*8
  self.contact_candidates=self.contact_candidate_type.field(shape=(self.max_contact_candidates,))
  self.max_contact_pairs=n
  self.contact_pairs=self.contact_pair_type.field(shape=(self.max_contact_pairs,))
  self.Jt=qd.field(gs.qd_vec3,shape=(n,self.rigid_solver.n_dofs_))
  self.M_inv_Jt=qd.field(gs.qd_vec3,shape=(n,self.rigid_solver.n_dofs_))
  self.W=qd.field(gs.qd_mat3,shape=(n,))
 sap.RigidFloorTetContactHandler.__init__=initialize
 sap.RigidRigidTetContactHandler.__init__=initialize
 sap.MAX_N_QUERY_RESULT_PER_AABB=query_factor
 sap.SAPCoupler.update_contact=safe_update_contact
