import numpy as np
import torch
import crm_torch

class ship_torch():
    def __init__(self):
        """
        classを初期化するmethod
        インスタンスを作成した際に必ず呼ばれる関数
        """
        self.x = None
        self.y = None
        self.psi = None
        self.K=0.060265768294965424
        self.T=30.249045960691753
        self.loa = 2.0

        self.U_min = 1*1852/3600
        self.U_max = 3*1852/3600

    def reset(self,x,y,U,psi,r):
        """
        位置(x,y)，速度U, 方位psiを初期化
        """
        self.x = x
        self.y = y
        self.U = U
        self.psi = psi
        self.r = r
        return self.data

    def move(self, r, acc, dt):
        """
        KTモデルの運動を計算
        """
        u_next = torch.clip(
            self.U + acc*dt,
            self.U_min, self.U_max
        )
        r_next = r
        psi_next = self.psi + (self.r+r_next)/2*dt
        x_next = self.x + (u_next+self.U)/2*torch.cos(torch.deg2rad((self.psi+psi_next)/2))*dt
        y_next = self.y + (u_next+self.U)/2*torch.sin(torch.deg2rad((self.psi+psi_next)/2))*dt

        self.x = x_next
        self.y = y_next
        self.psi = psi_next
        self.r = r_next
        self.U = u_next

        return self.data
    
    @property
    def data(self):
        """
        propoatyで定義されたmethodは，(インスタンス名).dataでreturnに書かれたデータを取得できる.
        関数を変数の様に扱うことができる．
        classの持っている変数に何か処理をしたデータが欲しい時などに便利
        """
        return [self.x, self.y, self.U, self.psi, self.r]

def diviation2path(x,y, p0, p1):
    p0top1 = np.sqrt( (p1[0]-p0[0])**2 + (p1[1]-p0[1])**2 )

    dist2p0_square = (x-p0[0])**2+(y-p0[1])**2
    naiseki =(
        (
            (x-p0[0])*(p1[0]-p0[0]) + (y-p0[1])*(p1[1]-p0[1])
        )/p0top1
    )
    _zeros = torch.zeros_like(x)
    return (
        torch.where(
            naiseki<0, torch.sqrt(dist2p0_square), _zeros
        ) + torch.where(
            (0<=naiseki)&(naiseki<=p0top1),
            torch.sqrt( torch.clip(dist2p0_square - naiseki**2,0,None) ), _zeros
        ) + torch.where(
            naiseki>=p0top1, torch.sqrt((x-p1[0])**2+(y-p1[1])**2), _zeros
        )
    )



class MPPItorch():
    def __init__(self,horizon,pred_dt,nb_sample,sigma,_lambda,umin,umax):
        self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.horizon = horizon
        self.pred_dt = pred_dt
        self.nb_sample = nb_sample
        self.sigma = sigma
        self._lambda = _lambda

        self.umin = umin
        self.umax = umax

        self._caldt = 0.1
        self._nbcalloop = int(self.pred_dt/self._caldt)
        self._caldt = self.pred_dt/self._nbcalloop

        self.ships_mppi = ship_torch()
    
    def prediction(self, x, y, psi, U, r, rudders, accs, seed=None, straight_time = 0):
        xs = torch.full((self.nb_sample,1),x).to(self._device)
        ys = torch.full((self.nb_sample,1),y).to(self._device)
        psis = torch.full((self.nb_sample,1),psi).to(self._device)
        speeds = torch.full((self.nb_sample,1),U).to(self._device)
        rs = torch.full((self.nb_sample,1),r).to(self._device)
        
        self.ships_mppi.reset(
            xs, ys, speeds, psis, rs
        )
        if seed is not None:
            torch.manual_seed(seed)
        
        straight_time = straight_time
        pred_rudders = torch.clip(
            torch.normal(
                mean=torch.tile(
                    torch.tensor(rudders).to(self._device), (self.nb_sample,1)
                ), std=torch.where(
                    torch.arange(rudders.shape[0])*self._caldt < straight_time,
                    torch.zeros(rudders.shape), torch.ones(rudders.shape)*self.sigma[0]
                ).to(self._device)
            ), self.umin[0], self.umax[0]
        )
        pred_rudders_offset = pred_rudders.mean(axis=-1,keepdim=True)
        # pred_rudders -= pred_rudders_offset
        pred_accs = torch.clip(
            torch.normal(
                mean=torch.tile(
                    torch.tensor(accs).to(self._device), (self.nb_sample,1)
                ), std=torch.where(
                    torch.arange(accs.shape[0])*self._caldt < straight_time,
                    torch.zeros(accs.shape), torch.ones(accs.shape)*self.sigma[1]
                ).to(self._device)
            ), self.umin[1], self.umax[1]
        )
        

        preds = [
            [self.ships_mppi.x,],
            [self.ships_mppi.y,],
            [self.ships_mppi.U,],
            [self.ships_mppi.psi,],
            [self.ships_mppi.r,],
            [torch.zeros_like(self.ships_mppi.r)]
        ]
        sim_time = 0
        for i in range(self.horizon):
            for _ in range(self._nbcalloop):
                sim_time += self._caldt
                self.ships_mppi.move(
                    pred_rudders[:,i:i+1],
                    pred_accs[:,i:i+1],
                    self._caldt
                )
                preds[0].append(self.ships_mppi.x)
                preds[1].append(self.ships_mppi.y)
                preds[2].append(self.ships_mppi.U)
                preds[3].append(self.ships_mppi.psi)
                preds[4].append(self.ships_mppi.r)
                preds[5].append(torch.ones_like(self.ships_mppi.r)*sim_time)
        
        predx   = torch.concatenate(preds[0], axis=-1)
        predy   = torch.concatenate(preds[1], axis=-1)
        predU   = torch.concatenate(preds[2], axis=-1)
        predpsi = torch.concatenate(preds[3], axis=-1)
        predr   = torch.concatenate(preds[4], axis=-1)
        predtimes = torch.concatenate(preds[5], axis=-1)

        return (
            predx, predy, predU, predpsi, predr, pred_rudders, pred_accs, predtimes
        )
    
    def get_opt(self, x, y, psi, U, r, rudders, accs, loa, others, path, col_cost_min, col_cost_max, div_cost, speed_cost, norm_cost, seed=None,straight_time = 0, debug=False):

        predx, predy, predU, predpsi, predr, pred_rudders, pred_accs, predtimes = self.prediction(
            x, y, psi, U, r, rudders, accs, seed=seed, straight_time = straight_time
        )

        # costs for collision risks
        cost_collision = crm_torch.timedomaincrm(
            predy,
            predx, 
            predtimes, # torch.sqrt((predy-y)**2+(predx-x)**2)/U,
            [y, x, U, psi, loa],
            [
                [oth.y, oth.x, oth.U, oth.psi, oth.loa]
                for oth in others
            ], turn=torch.deg2rad(predpsi)
        )
        
        # cost_collision_gain = torch.exp( -predtimes**2/(600)**2/2 )
        # cost_collision_gain = torch.where(
        #     predtimes<=predtimes.max()*0.60, torch.ones_like(predtimes), torch.zeros_like(predtimes)
        # )

        cost_collision = cost_collision.sum(axis=-1, keepdim=True)
        # cost_collision = (cost_collision_gain*cost_collision).sum(axis=-1, keepdim=True)
        # cost_collision = (cost_collision).sum(axis=-1, keepdim=True)

        # cost_collision = cost_collision/(1+torch.abs(cost_collision))
        cost_collision = torch.where(
            cost_collision>0, col_cost_min+(col_cost_max-col_cost_min)*cost_collision, cost_collision
        )

        # costs for diviation from course
        targ_u = torch.full_like(predU, path[1][2])
        diviation = diviation2path(predx, predy, path[0][:2], path[1][:2])
        for i in range(2, len(path)):
            _div_new = diviation2path(predx, predy, path[i-1][:2], path[i][:2])
            targ_u = torch.where(
                diviation>=_div_new, torch.full_like(predU, path[i][2]), targ_u
            )
            diviation = torch.minimum( diviation, _div_new )

        diviation = diviation.mean(axis=-1, keepdim=True)

        # costs for difference bbetween pred u and target u
        speed_diff = torch.square( targ_u - predU )
        speed_diff = speed_diff.mean(axis=-1, keepdim=True)


        if torch.isnan(cost_collision).any():
            print('[MPPI] cost計算が発散しています. cost for collision',)
        if torch.isnan(diviation).any():
            print('[MPPI] cost計算が発散しています. cost for div',)
        if torch.isnan(speed_diff).any():
            print('[MPPI] cost計算が発散しています. cost for speed diff',)
        
        without_collision = False
        if without_collision:
            costs = (
                div_cost*diviation
                + speed_cost*speed_diff
            )
            # norm costs        
            norm_targ = ['input', 'min'][1]
            if norm_targ == 'input':
                rudders_norm = torch.tensor(rudders).to(self._device)
                accs_norm = torch.tensor(accs).to(self._device)
            elif norm_targ == 'min':
                min_idx = torch.argmin(costs.flatten())
                rudders_norm = pred_rudders[min_idx]
                accs_norm = pred_accs[min_idx]


            costs_norm = (
                norm_cost*(torch.square(pred_rudders-rudders_norm)/self.umax[0]).mean(axis=-1, keepdim=True)
                + norm_cost*(torch.square(pred_accs-accs_norm)/self.umax[1]).mean(axis=-1, keepdim=True)
            )
            
            # costs_norm is added to costs
            costs = costs + costs_norm
            
            # calculate weights for trajectories
            cost_collision_min = cost_collision.min() #.values
            cost_collision_max = cost_collision.max() #.values
            th = 0
            w = torch.where(
                cost_collision<=th,
                torch.exp(-1/self._lambda*costs),
                torch.zeros_like(costs)
            )
            if w.sum()>0:
                w = w/w.sum()
            else:
                w = torch.where(
                    cost_collision<=cost_collision_min+(cost_collision_max-cost_collision_min)*0.1,
                    torch.exp(-1/self._lambda*costs),
                    torch.zeros_like(costs)
                )

        else:
            # calculate total costs
            costs = (
                cost_collision
                + div_cost*diviation
                + speed_cost*speed_diff
            )
            # norm costs        
            norm_targ = ['input', 'min'][0]
            if norm_targ == 'input':
                rudders_norm = torch.tensor(rudders).to(self._device)
                accs_norm = torch.tensor(accs).to(self._device)
            elif norm_targ == 'min':
                min_idx = torch.argmin(costs.flatten())
                rudders_norm = pred_rudders[min_idx]
                accs_norm = pred_accs[min_idx]


            costs_norm = (
                norm_cost*(torch.square(pred_rudders-rudders_norm)/self.umax[0]).mean(axis=-1, keepdim=True)
                + norm_cost*(torch.square(pred_accs-accs_norm)/self.umax[1]).mean(axis=-1, keepdim=True)
            )

            # costs_norm is added to costs
            costs = costs + costs_norm
            
            # calculate weights for trajectories
            w = torch.exp(-1/self._lambda*costs)
            w = w/w.sum()

        # # calculate optimal input
        # opt_rudder = (pred_rudders*w).sum(axis=0)
        # opt_acc = (pred_accs*w).sum(axis=0)
        opt_rudder = pred_rudders[torch.argmax(w.flatten())]
        opt_acc = pred_accs[torch.argmax(w.flatten())]

        if debug:
            return (
                predx.cpu().numpy().copy(),
                predy.cpu().numpy().copy(),
                predU.cpu().numpy().copy(),
                predpsi.cpu().numpy().copy(),
                predr.cpu().numpy().copy(),
                pred_rudders.cpu().numpy().copy(),
                pred_accs.cpu().numpy().copy(),
                costs.cpu().numpy().copy(),
                w.cpu().numpy().copy(),
                opt_rudder.cpu().numpy().copy(),
                opt_acc.cpu().numpy().copy(),
                cost_collision.cpu().numpy().copy(),
            )
        else:
            return (
                opt_rudder.cpu().numpy().copy(),
                opt_acc.cpu().numpy().copy(),
            )